#!/usr/bin/env python3
"""Audit physical grid neighbours from an evaluator observations CSV."""
import argparse
import csv
import sys
from pathlib import Path
try:
    from led_spacing import evaluate, grid_pairs, save, scorecard, parse_excluded_ids, load_availability
except ModuleNotFoundError as exc:
    raise SystemExit(f'Missing dependency {exc.name}. Run: python3 -m pip install -r evaluation/requirements.txt')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('observations_csv',type=Path)
    p.add_argument('--columns',type=int,required=True)
    p.add_argument('--rows',type=int,required=True)
    p.add_argument('--first-id',type=int,default=0)
    p.add_argument('--id-map-csv',type=Path,help='Complete mapping: beacon_id,row,column')
    p.add_argument('--spacing-x-mm',type=float,default=120.)
    p.add_argument('--spacing-y-mm',type=float,default=120.)
    p.add_argument('--pair-reference-csv',type=Path)
    p.add_argument('--max-relative-rmse',type=float,default=2.)
    p.add_argument('--output-dir',type=Path,default=Path('spacing_evaluation'))
    p.add_argument('--output',type=Path,help='Compatibility alias for per-pair CSV output')
    p.add_argument('--exclude-led-ids', default='none', help='Optional additional manual exclusions; availability filtering is automatic')
    p.add_argument('--availability-json',type=Path,help='Upstream camera1_ids/camera2_ids lists')
    args=p.parse_args()
    import math
    if not math.isfinite(args.max_relative_rmse) or args.max_relative_rmse<0:
        p.error('--max-relative-rmse must be finite and nonnegative')
    print('Reading observations and checking physical LED neighbours...',flush=True)
    with args.observations_csv.open(newline='') as f: obs=list(csv.DictReader(f))
    refs={}
    if args.pair_reference_csv:
        with args.pair_reference_csv.open(newline='') as f:
            for row in csv.DictReader(f):
                key=tuple(sorted((int(row['led_a']),int(row['led_b']))))
                if key in refs or key[0]==key[1]: raise ValueError('Duplicate or self pair reference')
                refs[key]=float(row['reference_length_mm'])
    result=evaluate(obs,grid_pairs(args.columns,args.rows,args.first_id,args.id_map_csv),
                    args.spacing_x_mm,args.spacing_y_mm,refs,parse_excluded_ids(args.exclude_led_ids),load_availability(args.availability_json))
    save(result,args.output_dir)
    if args.output:
        from led_spacing import write_csv
        write_csv(args.output,result['per_pair'])
    text=scorecard(result,args.max_relative_rmse)
    (args.output_dir/'terminal_scorecard.txt').write_text(text)
    print(text,flush=True)
    print(f'Outputs: {args.output_dir.resolve()}',flush=True)


if __name__=='__main__':
    try: main()
    except (ValueError,KeyError,OSError) as exc:
        print(f'ERROR: {exc}',file=sys.stderr,flush=True);sys.exit(2)
