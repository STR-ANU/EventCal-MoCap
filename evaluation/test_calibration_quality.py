#!/usr/bin/env python3
"""Synthetic numerical tests for calibration_quality.py."""

from __future__ import annotations

import unittest

import cv2
import numpy as np

from calibration_quality import (
    adjacent_led_pairs,
    distribution,
    epipolar_errors,
    fundamental_from_rt,
    triangulate_and_score,
)


class CalibrationQualityTests(unittest.TestCase):
    def setUp(self) -> None:
        self.K = np.array(
            [[800.0, 0.0, 640.0], [0.0, 800.0, 360.0], [0.0, 0.0, 1.0]]
        )
        self.distortion = np.zeros(5)
        self.R = np.eye(3)
        self.T = np.array([-0.25, 0.0, 0.0])
        self.points3d = np.array(
            [
                [-0.20, -0.10, 2.0],
                [0.10, -0.10, 2.4],
                [-0.15, 0.20, 3.0],
                [0.25, 0.15, 3.5],
            ],
            dtype=np.float64,
        )

    def project(self) -> tuple[np.ndarray, np.ndarray]:
        p1, _ = cv2.projectPoints(
            self.points3d, np.zeros(3), np.zeros(3), self.K, self.distortion
        )
        p2, _ = cv2.projectPoints(
            self.points3d, np.zeros(3), self.T, self.K, self.distortion
        )
        return p1.reshape(1, -1, 2), p2.reshape(1, -1, 2)

    def test_perfect_geometry(self) -> None:
        p1, p2 = self.project()
        result = triangulate_and_score(
            p1,
            p2,
            self.K,
            self.distortion,
            self.K,
            self.distortion,
            self.R,
            self.T,
        )
        self.assertTrue(np.all(result["valid"]))
        self.assertLess(distribution(result["reproj1"])["rms"], 1e-8)
        self.assertLess(distribution(result["reproj2"])["rms"], 1e-8)
        self.assertLess(distribution(result["sampson"])["rms"], 1e-8)
        np.testing.assert_allclose(result["points3d"][0], self.points3d, atol=1e-9)

    def test_epipolar_perturbation_is_detected(self) -> None:
        p1, p2 = self.project()
        F = fundamental_from_rt(self.K, self.K, self.R, self.T)
        perfect, _ = epipolar_errors(p1.reshape(-1, 2), p2.reshape(-1, 2), F)
        perturbed = p2.reshape(-1, 2).copy()
        perturbed[:, 1] += 2.0
        shifted, _ = epipolar_errors(p1.reshape(-1, 2), perturbed, F)
        self.assertLess(float(np.max(perfect)), 1e-10)
        self.assertGreater(float(np.median(shifted)), 1.0)

    def test_points_behind_camera_are_rejected(self) -> None:
        behind = np.array([[0.0, 0.0, -2.0]], dtype=np.float64)
        p1, _ = cv2.projectPoints(
            behind, np.zeros(3), np.zeros(3), self.K, self.distortion
        )
        p2, _ = cv2.projectPoints(
            behind, np.zeros(3), self.T, self.K, self.distortion
        )
        result = triangulate_and_score(
            p1.reshape(1, 1, 2),
            p2.reshape(1, 1, 2),
            self.K,
            self.distortion,
            self.K,
            self.distortion,
            self.R,
            self.T,
        )
        self.assertFalse(bool(result["valid"][0, 0]))


class AdjacentLedPairsTests(unittest.TestCase):
    """M6: adjacent-LED spacing consistency reuses length_metrics() with a
    different pair list, so what needs its own coverage is the pair list
    itself - adjacent_led_pairs()."""

    def test_full_grid_pair_count_and_directions(self) -> None:
        # A 4-column x 6-row board has 3 horizontal neighbors per row (6 rows)
        # and 5 vertical neighbors per column (4 columns): 18 + 20 = 38 edges.
        beacon_ids = list(range(24))
        pairs = adjacent_led_pairs(beacon_ids, columns=4, rows=6, h5_id_for_led1=0)
        self.assertEqual(len(pairs), 38)
        self.assertEqual(sum(1 for _, _, d in pairs if d == "horizontal"), 18)
        self.assertEqual(sum(1 for _, _, d in pairs if d == "vertical"), 20)
        # Every pair must actually be one grid step apart.
        for a, b, direction in pairs:
            if direction == "horizontal":
                self.assertEqual(b - a, 1)
                self.assertEqual(a // 4, b // 4)  # same row
            else:
                self.assertEqual(b - a, 4)

    def test_missing_beacons_drop_only_their_touching_edges(self) -> None:
        # Beacon 14 never detected: every pair touching it should disappear,
        # everything else should be unaffected.
        beacon_ids = [b for b in range(24) if b != 14]
        pairs = adjacent_led_pairs(beacon_ids, columns=4, rows=6, h5_id_for_led1=0)
        self.assertEqual(len(pairs), 38 - 4)  # 14 has 4 grid neighbors: 13,15,10,18
        self.assertTrue(all(14 not in (a, b) for a, b, _ in pairs))

    def test_h5_id_offset_is_applied(self) -> None:
        # If the firmware's first LED isn't beacon_id 0, pairs must be
        # generated in that offset id-space, not always starting at 0.
        beacon_ids = list(range(100, 124))
        pairs = adjacent_led_pairs(beacon_ids, columns=4, rows=6, h5_id_for_led1=100)
        self.assertEqual(len(pairs), 38)
        self.assertTrue(all(a >= 100 and b >= 100 for a, b, _ in pairs))


if __name__ == "__main__":
    unittest.main()