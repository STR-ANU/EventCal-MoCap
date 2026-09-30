"""Regression tests for metric validity, missing data and visible CLI output."""
import csv
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
import numpy as np
import cv2
from led_spacing import grid_pairs, evaluate
from calibration_quality import triangulate_and_score, camera_parameters

HERE=Path(__file__).resolve().parent


def observations(scale=1.,missing=None):
    return [dict(view_index=v,beacon_id=b,valid_3d=1,
                 X_cam1_mm=(b%4)*120*scale,Y_cam1_mm=(b//4)*120*scale,Z_cam1_mm=2000.)
            for v in range(2) for b in range(24) if b!=missing]


class SpacingTests(unittest.TestCase):
    def test_perfect_full_grid(self):
        result=evaluate(observations(),grid_pairs(4,6))
        self.assertEqual(result['planned_pairs'],38)
        self.assertEqual(result['aggregates']['all']['valid_count'],76)
        self.assertEqual(result['aggregates']['all']['rmse_mm'],0.)

    def test_scale_bias_detected(self):
        result=evaluate(observations(1.1),grid_pairs(4,6))
        self.assertAlmostEqual(result['aggregates']['all']['bias_mm'],12.)
        self.assertAlmostEqual(result['aggregates']['all']['relative_rmse_percent'],10.)

    def test_missing_led_automatically_excluded(self):
        result=evaluate(observations(missing=0),grid_pairs(4,6))
        self.assertEqual(result['evaluable_pairs'],36)
        self.assertEqual(result['aggregates']['all']['invalid_count'],0)
        self.assertEqual(result['planned_pairs'],36)

    def test_excluded_ids_do_not_count_as_missing(self):
        rows=[r for r in observations() if r['beacon_id'] not in (14,21)]
        result=evaluate(rows,grid_pairs(4,6),excluded_ids={14,21})
        self.assertEqual(result['planned_pairs'],31)
        self.assertEqual(result['evaluable_pairs'],31)
        self.assertEqual(len(result['excluded_pairs']),7)
        self.assertEqual(result['aggregates']['all']['invalid_count'],0)
        self.assertTrue(all(r['led_a'] not in (14,21) and r['led_b'] not in (14,21)
                            for r in result['per_observation']))

    def test_arbitrary_upstream_missing_ids(self):
        result=evaluate(observations(),grid_pairs(4,6),availability=(set(range(24))-{5},set(range(24))-{9}))
        self.assertEqual(result['availability_by_run'][0]['automatically_excluded_ids'],[5,9])
        self.assertTrue(all(r['led_a'] not in (5,9) and r['led_b'] not in (5,9) for r in result['per_observation']))
        self.assertEqual(result['aggregates']['all']['invalid_count'],0)

    def test_temporary_dropout_does_not_remove_led(self):
        rows=observations()
        rows=[r for r in rows if not (r['view_index']==0 and r['beacon_id']==0)]
        result=evaluate(rows,grid_pairs(4,6))
        self.assertEqual(result['planned_pairs'],38)
        self.assertEqual(result['aggregates']['all']['invalid_count'],2)

    def test_duplicate_rejected(self):
        rows=observations();rows.append(rows[0])
        with self.assertRaisesRegex(ValueError,'Duplicate'): evaluate(rows,grid_pairs(4,6))

    def test_no_cross_run_pairing(self):
        a=observations(missing=0);b=observations(missing=1)
        for row in a: row['run_id']='a'
        for row in b: row['run_id']='b'
        r=evaluate(a+b,[(0,1,'horizontal')])
        self.assertEqual(r['aggregates']['all']['valid_count'],0)

    def test_nonzero_start(self):
        rows=observations()
        for x in rows: x['beacon_id']+=10
        self.assertEqual(evaluate(rows,grid_pairs(4,6,10))['aggregates']['all']['rmse_mm'],0.)

    def test_custom_mapping(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'map.csv';p.write_text('beacon_id,row,column\n7,0,0\n3,0,1\n')
            self.assertEqual(grid_pairs(2,1,mapping_path=p),[(7,3,'horizontal')])

    def test_nonfinite_point_counted_invalid(self):
        rows=observations();rows[0]['X_cam1_mm']=float('nan')
        r=evaluate(rows,grid_pairs(4,6))
        self.assertEqual(r['aggregates']['all']['invalid_count'],2)

    def test_invalid_reference(self):
        with self.assertRaises(ValueError): evaluate(observations(),grid_pairs(4,6),spacing_x_mm=0.)


class CliTests(unittest.TestCase):
    def fixture(self,d):
        K=np.array([[800.,0,640],[0,800,360],[0,0,1]])
        R=np.eye(3);T=np.array([-.3,0.,0.])
        xyz=np.array([[c*.12,r*.12,3.] for r in range(6) for c in range(4)])
        p1=cv2.projectPoints(xyz,np.zeros(3),np.zeros(3),K,np.zeros(5))[0].reshape(1,-1,2)
        p2=cv2.projectPoints(xyz,np.zeros(3),T,K,np.zeros(5))[0].reshape(1,-1,2)
        np.savez(d/'points.npz',img_points_cam1=p1,img_points_cam2=p2,beacon_ids=np.arange(24),sample_times_cam1=np.array([0.]))
        stereo=dict(R=R.tolist(),T=T.tolist(),geometry=dict(columns=4,rows=6,spacing_x=.12,spacing_y=.12,h5_id_for_led1=0,is_real_scale=True))
        (d/'stereo.json').write_text(json.dumps(stereo))
        intr=dict(camera_matrix=K.tolist(),distortion_coefficients=[0]*5)
        (d/'intr.json').write_text(json.dumps(intr))
        return [str(d/'points.npz'),str(d/'stereo.json'),str(d/'intr.json'),str(d/'intr.json'),'--output-dir',str(d/'out')]

    def test_cli_scorecard_strict_and_missing_evidence(self):
        with tempfile.TemporaryDirectory() as folder:
            d=Path(folder);args=self.fixture(d)
            r=subprocess.run([sys.executable,str(HERE/'calibration_quality.py'),*args,'--strict'],capture_output=True,text=True)
            self.assertEqual(r.returncode,1,r.stderr)
            self.assertIn('CALIBRATION QUALITY SCORECARD',r.stdout)
            self.assertIn('NOT_EVALUATED',r.stdout)
            self.assertTrue((d/'out/terminal_scorecard.txt').is_file())
            report=json.loads((d/'out/evaluation_report.json').read_text())
            self.assertNotIn('sqrt_sampson_rms',report['decision']['gates'])
            self.assertLess(report['adjacent_spacing']['aggregates']['all']['rmse_mm'],1e-8)
            self.assertIn('input_sha256',report)

    def test_legacy_sampson_gate_opt_in(self):
        with tempfile.TemporaryDirectory() as folder:
            d=Path(folder);args=self.fixture(d)
            r=subprocess.run([sys.executable,str(HERE/'calibration_quality.py'),*args,'--gate-sampson'],capture_output=True,text=True)
            self.assertEqual(r.returncode,0,r.stderr)
            report=json.loads((d/'out/evaluation_report.json').read_text())
            self.assertIn('sqrt_sampson_rms',report['decision']['gates'])

    def test_npz_decoded_availability_filters_every_metric(self):
        with tempfile.TemporaryDirectory() as folder:
            d=Path(folder);args=self.fixture(d)
            with np.load(d/'points.npz') as f: data={k:f[k] for k in f.files}
            data['decoded_beacon_ids_cam1']=np.array([b for b in range(24) if b!=5])
            data['decoded_beacon_ids_cam2']=np.array([b for b in range(24) if b!=9])
            np.savez(d/'points.npz',**data)
            r=subprocess.run([sys.executable,str(HERE/'calibration_quality.py'),*args],capture_output=True,text=True)
            self.assertEqual(r.returncode,0,r.stderr)
            report=json.loads((d/'out/evaluation_report.json').read_text())
            self.assertEqual(report['id_availability']['automatically_excluded_ids'],[5,9])
            self.assertEqual(report['conditional_reconstruction']['attempted'],22)
            self.assertNotIn('one_or_more_physical_adjacent_pairs_unobserved',report['decision']['missing_required_evidence'])
            self.assertNotIn(5,report['id_availability']['evaluated_ids'])
            self.assertNotIn(9,report['id_availability']['evaluated_ids'])

    def test_empty_npz_fails_loudly(self):
        with tempfile.TemporaryDirectory() as folder:
            d=Path(folder);args=self.fixture(d)
            np.savez(d/'points.npz',img_points_cam1=np.empty((0,24,2)),img_points_cam2=np.empty((0,24,2)),beacon_ids=np.arange(24),sample_times_cam1=np.array([]))
            r=subprocess.run([sys.executable,str(HERE/'calibration_quality.py'),*args],capture_output=True,text=True)
            self.assertEqual(r.returncode,2)
            self.assertIn('ERROR:',r.stderr)

    def test_standalone_csv_cli(self):
        with tempfile.TemporaryDirectory() as folder:
            d=Path(folder);rows=observations()
            with (d/'obs.csv').open('w',newline='') as f:
                w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
            r=subprocess.run([sys.executable,str(HERE/'adjacent_led_spacing_check.py'),str(d/'obs.csv'),'--columns','4','--rows','6','--output-dir',str(d/'out')],capture_output=True,text=True)
            self.assertEqual(r.returncode,0,r.stderr)
            self.assertIn('38/38',r.stdout)
            self.assertIn('Automatically excluded IDs (unavailable in either camera): []',r.stdout)


class GeometryValidationTests(unittest.TestCase):
    def test_frozen_stereo_values_need_no_intrinsic_fallback(self):
        K,d=camera_parameters({},dict(camera_matrix_1=np.eye(3).tolist(),distortion_1=[0]*5),1)
        self.assertEqual(K.shape,(3,3))

    def test_invalid_baseline(self):
        p=np.zeros((1,1,2));K=np.eye(3)
        with self.assertRaisesRegex(ValueError,'baseline'): triangulate_and_score(p,p,K,np.zeros(5),K,np.zeros(5),K,np.zeros(3))

    def test_nonfinite_image_input(self):
        K=np.array([[800.,0,640],[0,800,360],[0,0,1]])
        p1=np.array([[[640.,360.],[float('nan'),360.]]]);p2=np.array([[[520.,360.],[520.,360.]]])
        r=triangulate_and_score(p1,p2,K,np.zeros(5),K,np.zeros(5),np.eye(3),np.array([-.3,0,0]))
        self.assertTrue(r['valid'][0,0]);self.assertFalse(r['valid'][0,1])
        self.assertEqual(r['failure_code'][0,1],'nonfinite_image_coordinate')


if __name__=='__main__': unittest.main()
