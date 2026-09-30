"""Adjacent LED spacing checks; NumPy + standard library, no board fitting."""
from __future__ import annotations
import csv
import json
from pathlib import Path
import numpy as np


def grid_pairs(columns, rows, first_id=0, mapping_path=None):
    if columns < 1 or rows < 1 or columns * rows < 2:
        raise ValueError('Grid must contain at least two LEDs and positive dimensions')
    mapping = {(r, c): first_id + r * columns + c for r in range(rows) for c in range(columns)}
    if mapping_path:
        with Path(mapping_path).open(newline='') as f:
            records = list(csv.DictReader(f))
        mapping = {}
        ids = set()
        for x in records:
            r, c, b = int(x['row']), int(x['column']), int(x['beacon_id'])
            if not (0 <= r < rows and 0 <= c < columns) or (r,c) in mapping or b in ids:
                raise ValueError('Mapping has duplicate/out-of-range grid cells or duplicate beacon IDs')
            mapping[r,c] = b
            ids.add(b)
        if len(mapping) != rows * columns:
            raise ValueError('Mapping must describe the complete physical board, including undetected LEDs')
    pairs = []
    for (r,c), a in mapping.items():
        for dr,dc,direction in [(0,1,'horizontal'), (1,0,'vertical')]:
            if (r+dr,c+dc) in mapping:
                pairs.append((a,mapping[r+dr,c+dc],direction))
    return pairs


def stats(errors, references, lengths):
    e = np.asarray(errors, dtype=float)
    if not len(e):
        return dict(valid_count=0, mean_distance_mm=None, bias_mm=None, mae_mm=None,
                    rmse_mm=None, relative_rmse_percent=None, sample_sd_mm=None,
                    p95_absolute_error_mm=None, maximum_absolute_error_mm=None)
    rel = e / np.asarray(references)
    return dict(valid_count=len(e), mean_distance_mm=float(np.mean(lengths)),
                bias_mm=float(e.mean()), mae_mm=float(np.abs(e).mean()),
                rmse_mm=float(np.sqrt(np.mean(e**2))),
                relative_rmse_percent=float(100*np.sqrt(np.mean(rel**2))),
                sample_sd_mm=float(e.std(ddof=1)) if len(e)>1 else None,
                p95_absolute_error_mm=float(np.percentile(np.abs(e),95)),
                maximum_absolute_error_mm=float(np.max(np.abs(e))))


def parse_excluded_ids(value):
    if value.strip().lower() in ('none', ''):
        return set()
    try:
        return {int(x.strip()) for x in value.split(',')}
    except ValueError as exc:
        raise ValueError('Excluded LED IDs must be comma-separated integers or none') from exc


def load_availability(path):
    if path is None:
        return None
    data = json.loads(Path(path).read_text())
    return (validated_ids(data['camera1_ids']), validated_ids(data['camera2_ids']))


def validated_ids(values):
    a = np.asarray(values)
    if a.ndim != 1 or (a.size and (not np.issubdtype(a.dtype, np.number) or
       not np.all(np.isfinite(a)) or not np.all(a == np.floor(a)))):
        raise ValueError('Camera ID lists must contain integer IDs')
    return {int(x) for x in a}


def evaluate(observations, pairs, spacing_x_mm=120., spacing_y_mm=120., measured=None, excluded_ids=None, availability=None):
    observations = list(observations)
    excluded_ids = set(excluded_ids or ())
    excluded_pairs = [f'{a}-{b}' for a,b,_ in pairs if a in excluded_ids or b in excluded_ids]
    pairs = [(a,b,d) for a,b,d in pairs if a not in excluded_ids and b not in excluded_ids]
    measured = measured or {}
    if not all(np.isfinite(x) and x > 0 for x in [spacing_x_mm,spacing_y_mm]):
        raise ValueError('LED spacings must be finite and positive')
    lookup = {}
    views = set()
    camera_ids = {}
    for o in observations:
        # run_id and partition prevent accidental cross-session pairing in merged CSVs.
        key = (str(o.get('run_id','')), str(o.get('partition','')), int(o['view_index']))
        b = int(o['beacon_id'])
        if (key,b) in lookup:
            raise ValueError(f'Duplicate observation for {key}, LED {b}; do not average silently')
        views.add(key)
        session = key[:2]
        ids1, ids2 = camera_ids.setdefault(session, (set(), set()))
        # Availability is established upstream or from 2D observations, never
        # by testing whether triangulation or a distance gate succeeded.
        for fields, ids in [(('u1','v1'),ids1),(('u2','v2'),ids2)]:
            if all(c in o for c in fields):
                if all(o[c] not in (None,'') and np.isfinite(float(o[c])) for c in fields): ids.add(b)
            else:
                ids.add(b)
        validity = str(o['valid_3d']).strip().lower()
        if validity not in ('0','1','false','true'):
            raise ValueError('valid_3d must be 0/1 or false/true')
        xyz = np.array([float('nan') if o.get(c) in (None, '') else float(o[c]) for c in ['X_cam1_mm','Y_cam1_mm','Z_cam1_mm']])
        lookup[key,b] = xyz if validity in ('1','true') and np.all(np.isfinite(xyz)) else None
    if not views:
        raise ValueError('No observations to evaluate')
    eligible = {}
    availability_report = []
    board_ids = {b for a,b,_ in pairs} | {a for a,b,_ in pairs}
    for session,(ids1,ids2) in camera_ids.items():
        if availability is not None:
            ids1,ids2 = ids1 & availability[0], ids2 & availability[1]
        eligible[session] = (ids1 & ids2) - excluded_ids
        availability_report.append(dict(run_id=session[0],partition=session[1],
            camera1_available_ids=sorted(ids1),camera2_available_ids=sorted(ids2),
            eligible_ids=sorted(eligible[session]),
            automatically_excluded_ids=sorted(board_ids-(ids1 & ids2)),
            source='upstream_metadata_and_observations' if availability is not None else 'supplied_observations'))
    automatic_pairs = [f'{a}-{b}' for a,b,_ in pairs if not any(a in ids and b in ids for ids in eligible.values())]
    pairs = [(a,b,d) for a,b,d in pairs if any(a in ids and b in ids for ids in eligible.values())]
    samples, summary = [], []
    for a,b,direction in pairs:
        pair = tuple(sorted((a,b)))
        ref = measured.get(pair, spacing_x_mm if direction=='horizontal' else spacing_y_mm)
        source = 'measured_pair_reference' if pair in measured else 'nominal_spacing'
        if not np.isfinite(ref) or ref <= 0:
            raise ValueError('Reference distances must be finite and positive')
        for key in sorted(views):
            if a not in eligible[key[:2]] or b not in eligible[key[:2]]:
                continue
            pa,pb = lookup.get((key,a)),lookup.get((key,b))
            valid = pa is not None and pb is not None
            distance = float(np.linalg.norm(pa-pb)) if valid else None
            error = distance-ref if valid else None
            samples.append(dict(run_id=key[0],partition=key[1],view_index=key[2],
                pair=f'{a}-{b}',led_a=a,led_b=b,direction=direction,valid=int(valid),
                failure_code='' if valid else 'missing_or_invalid_endpoint',
                reference_mm=ref,reference_source=source,distance_mm=distance,
                error_mm=error,relative_error_percent=100*error/ref if valid else None))
        subset = [x for x in samples if x['pair']==f'{a}-{b}']
        valid_subset = [x for x in subset if x['valid']]
        summary.append(dict(pair=f'{a}-{b}',direction=direction,nominal_mm=ref,
            reference_source=source,attempted_count=len(subset),invalid_count=len(subset)-len(valid_subset),
            **stats([x['error_mm'] for x in valid_subset], [x['reference_mm'] for x in valid_subset],
                    [x['distance_mm'] for x in valid_subset])))
    aggregates = {}
    for direction in ('horizontal','vertical','all'):
        subset = [x for x in samples if direction=='all' or x['direction']==direction]
        ok = [x for x in subset if x['valid']]
        aggregates[direction] = dict(attempted_count=len(subset),invalid_count=len(subset)-len(ok),
            **stats([x['error_mm'] for x in ok],[x['reference_mm'] for x in ok],[x['distance_mm'] for x in ok]))
    by_view = []
    for key in sorted(views):
        for direction in ('horizontal','vertical','all'):
            subset = [x for x in samples if (x['run_id'],x['partition'],x['view_index'])==key and
                      (direction=='all' or x['direction']==direction)]
            ok = [x for x in subset if x['valid']]
            by_view.append(dict(run_id=key[0],partition=key[1],view_index=key[2],direction=direction,
                attempted_count=len(subset),invalid_count=len(subset)-len(ok),
                **stats([x['error_mm'] for x in ok],[x['reference_mm'] for x in ok],[x['distance_mm'] for x in ok])))
    return dict(per_pair=summary,per_observation=samples,aggregates=aggregates,by_view=by_view,
                excluded_led_ids=sorted(excluded_ids),excluded_pairs=excluded_pairs + automatic_pairs,
                availability_by_run=availability_report,automatically_excluded_pairs=automatic_pairs,
                planned_pairs=len(pairs),evaluable_pairs=sum(x['valid_count']>0 for x in summary),
                warning='Conditional on supplied views; missing recordings/detections before CSV creation are not counted. '
                'Nominal board spacing is not independent scale ground truth. No rigid-board fitting or rescaling is applied.')


def write_csv(path, records):
    path = Path(path)
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('w',newline='') as f:
        if records:
            w=csv.DictWriter(f,fieldnames=list(records[0]));w.writeheader();w.writerows(records)


def save(result, output_dir):
    output_dir=Path(output_dir);output_dir.mkdir(parents=True,exist_ok=True)
    write_csv(output_dir/'adjacent_spacing_audit.csv',result['per_pair'])
    write_csv(output_dir/'adjacent_spacing_samples.csv',result['per_observation'])
    write_csv(output_dir/'adjacent_spacing_by_view.csv',result['by_view'])
    write_csv(output_dir/'adjacent_spacing_by_direction.csv',
              [dict(direction=k,**v) for k,v in result['aggregates'].items()])
    (output_dir/'adjacent_spacing_report.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')


def scorecard(result, max_relative_rmse=2.):
    automatic_ids = sorted({b for x in result.get('availability_by_run',[]) for b in x['automatically_excluded_ids']})
    lines=['ADJACENT LED SPACING (free 3D triangulation)',
           f'Automatically excluded IDs (unavailable in either camera): {automatic_ids}', 
           f"Excluded LED IDs: {result.get('excluded_led_ids', [])}; excluded pairs: {len(result.get('excluded_pairs', []))}",
           f"Evaluable pairs: {result['evaluable_pairs']}/{result['planned_pairs']}",
           'Direction     N     Mean(mm)    Bias(mm)    RMSE(mm)    Rel.RMSE(%)']
    for direction,s in result['aggregates'].items():
        def fmt(v): return 'N/A' if v is None else f'{v:.3f}'
        lines.append(f"{direction:<12} {s['valid_count']:>4} {fmt(s['mean_distance_mm']):>12} "
                     f"{fmt(s['bias_mm']):>11} {fmt(s['rmse_mm']):>11} {fmt(s['relative_rmse_percent']):>14}")
    value=result['aggregates']['all']['relative_rmse_percent']
    failed=value is not None and value>max_relative_rmse
    missing=result['evaluable_pairs']<result['planned_pairs'] or value is None
    status='FAILED' if failed else 'NOT_EVALUATED' if missing else 'WITHIN_CONFIGURED_SPACING_LIMIT'
    lines += [f'Spacing status: {status}; configured relative RMSE limit = {max_relative_rmse:g}%',
              'This spacing-only result is not a final calibration pass.',result['warning']]
    return '\n'.join(lines)+'\n'
