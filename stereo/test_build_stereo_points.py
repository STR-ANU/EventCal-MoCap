import unittest

import numpy as np

from build_stereo_points import build_stereo_image_points, interpolate_if_local


class InterpolateIfLocalTests(unittest.TestCase):
    def test_exact_match_returns_zero_speed(self):
        times = np.array([0.0, 1.0, 2.0])
        xy = np.array([[0.0, 0.0], [10.0, 0.0], [20.0, 0.0]])
        result = interpolate_if_local(times, xy, 1.0, max_gap_s=0.5)
        self.assertIsNotNone(result)
        point, speed = result
        np.testing.assert_allclose(point, [10.0, 0.0])
        self.assertEqual(speed, 0.0)

    def test_midpoint_interpolation_and_speed(self):
        # Two samples 1s apart, 100px apart in x -> speed 100 px/s, and the
        # query sits exactly halfway -> exactly the midpoint position.
        times = np.array([0.0, 1.0])
        xy = np.array([[0.0, 0.0], [100.0, 0.0]])
        point, speed = interpolate_if_local(times, xy, 0.5, max_gap_s=1.0)
        np.testing.assert_allclose(point, [50.0, 0.0])
        self.assertAlmostEqual(speed, 100.0)

    def test_gap_too_large_returns_none(self):
        times = np.array([0.0, 1.0])
        xy = np.array([[0.0, 0.0], [100.0, 0.0]])
        self.assertIsNone(interpolate_if_local(times, xy, 0.5, max_gap_s=0.1))

    def test_outside_range_returns_none(self):
        times = np.array([0.0, 1.0])
        xy = np.array([[0.0, 0.0], [100.0, 0.0]])
        self.assertIsNone(interpolate_if_local(times, xy, 5.0, max_gap_s=10.0))

    def test_near_zero_dt_does_not_blow_up_speed_from_noise(self):
        # Regression test for a real bug: two bracketing samples a fraction
        # of a millisecond apart (routine tracker timing jitter) with only a
        # couple of pixels of ordinary measurement noise between them must
        # NOT produce an absurd "speed" just because dt in the denominator
        # is tiny. This was observed in practice: reported speeds of
        # 48,000-560,000 px/s on a recording whose actual frame-to-frame
        # motion (at the 10 Hz output grid) was only 320-480 px/s.
        times = np.array([0.0, 0.0003, 0.5])  # 0.3ms gap between rows 0 and 1
        xy = np.array([[640.0, 360.0], [642.0, 360.5], [641.0, 360.0]])
        point, speed = interpolate_if_local(times, xy, 0.00015, max_gap_s=0.05)
        np.testing.assert_allclose(point, [641.0, 360.25])
        # With the default 2ms floor, ~2px of noise over a floored 2ms
        # denominator bounds speed to roughly 2px/0.002s = ~1000 px/s, not
        # the ~6872 px/s (2px / 0.0003s) the unfloored calculation gives.
        self.assertLess(speed, 2000.0)

    def test_min_speed_dt_floor_does_not_affect_interpolated_position(self):
        # The floor must only change the reported speed, never the actual
        # interpolated position - those are two different pieces of the
        # return value and must not be coupled.
        times = np.array([0.0, 0.0003, 0.5])
        xy = np.array([[640.0, 360.0], [642.0, 360.5], [641.0, 360.0]])
        point_default, _ = interpolate_if_local(times, xy, 0.00015, max_gap_s=0.05, min_speed_dt_s=0.002)
        point_no_floor, _ = interpolate_if_local(times, xy, 0.00015, max_gap_s=0.05, min_speed_dt_s=0.0)
        np.testing.assert_allclose(point_default, point_no_floor)

    def test_genuine_fast_motion_over_a_reasonable_gap_is_still_detected(self):
        # The floor must not mask real fast motion when dt is already well
        # above the floor - only the near-zero-dt/noise regime should be
        # affected.
        times = np.array([0.0, 0.02, 0.5])  # 20ms gap, well above the 2ms floor
        xy = np.array([[640.0, 360.0], [660.0, 360.0], [641.0, 360.0]])  # 20px / 20ms = 1000 px/s
        _, speed = interpolate_if_local(times, xy, 0.01, max_gap_s=0.05)
        self.assertAlmostEqual(speed, 1000.0, delta=1.0)


class BuildStereoImagePointsTests(unittest.TestCase):
    def _oscillating_tracks(self, amplitude_px=400.0, period_s=1.0, raw_hz=200.0, duration_s=10.0):
        t = np.arange(0, duration_s, 1.0 / raw_hz)
        x = 640 + amplitude_px * np.sin(2 * np.pi * t / period_s)
        y = 360 + 0.3 * amplitude_px * np.sin(2 * np.pi * t / period_s + 0.3)
        x2 = x + 50.0
        y2 = y + 20.0
        tracks1 = {0: (t, np.column_stack([x, y]))}
        tracks2 = {0: (t, np.column_stack([x2, y2]))}
        return tracks1, tracks2

    def test_default_behavior_unchanged_without_velocity_filter(self):
        # With max_velocity_px_per_s=None and min_points=1 (this fixture only
        # has one beacon), every candidate sample with valid, in-gap data on
        # both cameras is kept, regardless of speed.
        tracks1, tracks2 = self._oscillating_tracks()
        pts1, pts2, times, ids, speeds, rejects = build_stereo_image_points(
            tracks1, tracks2, a=1.0, b=0.0, hz=10.0, max_gap_s=0.05,
            max_velocity_px_per_s=None, min_points=1,
        )
        self.assertEqual(rejects["total_velocity_rejected_slots"], 0)
        self.assertGreater(len(pts1), 0)
        self.assertEqual(len(pts1), len(times))
        self.assertEqual(len(speeds), len(pts1))
        # The oscillation must actually produce some high-speed samples in
        # this fixture, or the rest of this test class isn't testing anything.
        self.assertGreater(speeds.max(), 500.0)

    def test_velocity_filter_bounds_kept_view_speed(self):
        tracks1, tracks2 = self._oscillating_tracks()
        threshold = 800.0
        pts1, pts2, times, ids, speeds, rejects = build_stereo_image_points(
            tracks1, tracks2, a=1.0, b=0.0, hz=10.0, max_gap_s=0.05,
            max_velocity_px_per_s=threshold, min_points=1,
        )
        # With only one beacon and min_points=1, a velocity-rejected slot
        # leaves that view with zero valid points, so it's ultimately
        # dropped for insufficient_points - but the slot-level counter still
        # attributes the underlying reason correctly.
        self.assertGreater(rejects["total_velocity_rejected_slots"], 0)
        self.assertGreater(rejects["insufficient_points"], 0)
        if len(speeds):
            self.assertLessEqual(speeds.max(), threshold)
        # Point count/order must stay aligned between cameras and across
        # kept views even when some candidate views are dropped.
        self.assertEqual(len(pts1), len(pts2))
        self.assertEqual(len(pts1), len(times))
        self.assertEqual(len(pts1), len(speeds))

    def test_stricter_threshold_rejects_more_or_equal_slots(self):
        tracks1, tracks2 = self._oscillating_tracks()
        _, _, _, _, _, loose = build_stereo_image_points(
            tracks1, tracks2, a=1.0, b=0.0, hz=10.0, max_gap_s=0.05,
            max_velocity_px_per_s=1500.0, min_points=1,
        )
        _, _, _, _, _, strict = build_stereo_image_points(
            tracks1, tracks2, a=1.0, b=0.0, hz=10.0, max_gap_s=0.05,
            max_velocity_px_per_s=500.0, min_points=1,
        )
        self.assertGreaterEqual(
            strict["total_velocity_rejected_slots"], loose["total_velocity_rejected_slots"]
        )

    def test_missing_and_velocity_rejections_are_counted_separately(self):
        # Beacon 0 is present and slow in both cameras (always keeps a view
        # alive on its own, since min_points=1 below). Beacon 1 is present
        # but fast in both cameras (some of its slots get velocity-rejected).
        # Beacon 2 is present in camera 1 but entirely absent from camera 2,
        # so it's excluded from usable_ids (the intersection) before any
        # per-sample check even runs - it must not show up as a gap
        # rejection at all, since it never had a chance to be missing at a
        # specific instant.
        t = np.arange(0, 2.0, 1.0 / 200.0)
        slow_xy = np.column_stack([np.full_like(t, 640.0), np.full_like(t, 360.0)])
        fast_xy = np.column_stack([640 + 1000.0 * np.sin(2 * np.pi * t), np.full_like(t, 360.0)])

        tracks1 = {0: (t, slow_xy), 1: (t, fast_xy), 2: (t, slow_xy)}
        tracks2 = {0: (t, slow_xy), 1: (t, fast_xy)}

        _, _, _, ids, _, rejects = build_stereo_image_points(
            tracks1, tracks2, a=1.0, b=0.0, hz=10.0, max_gap_s=0.05,
            max_velocity_px_per_s=200.0, min_points=1,
        )
        self.assertEqual(ids, [0, 1])
        self.assertGreater(rejects["total_velocity_rejected_slots"], 0)
        # Beacon 0 alone always satisfies min_points=1, so views should
        # almost never be dropped outright in this fixture - allow a small
        # count for benign boundary effects at the very edge of the
        # tracker's time range, rather than asserting an exact 0.
        self.assertLessEqual(rejects["insufficient_points"], 1)

    def _partially_occluded_tracks(self, num_beacons=22, duration_s=10.0, raw_hz=50.0):
        # Mirrors the real failure this fixes: a board with many LEDs where
        # each camera only ever sees a SUBSET at any one instant (occlusion
        # from tilting/sweeping), and the two cameras' visible subsets
        # aren't identical either - i.e. exactly what a real multi-LED board
        # with any pose diversity looks like, as opposed to the small
        # toy/2-3-beacon fixtures above.
        rng = np.random.default_rng(0)
        t = np.arange(0, duration_s, 1.0 / raw_hz)
        tracks1, tracks2 = {}, {}
        for beacon_id in range(num_beacons):
            # Each beacon is visible in a random ~60% window of the
            # recording in each camera, independently - so "all beacons
            # visible in both cameras at once" essentially never happens,
            # but "at least 12 visible at once" very often does.
            visible1 = rng.random(len(t)) < 0.6
            visible2 = rng.random(len(t)) < 0.6
            xy = np.column_stack([
                640 + 50 * np.sin(2 * np.pi * t / 3.0 + beacon_id),
                360 + 50 * np.cos(2 * np.pi * t / 3.0 + beacon_id),
            ])
            if visible1.any():
                tracks1[beacon_id] = (t[visible1], xy[visible1])
            if visible2.any():
                tracks2[beacon_id] = (t[visible2] , (xy + 10.0)[visible2])
        return tracks1, tracks2

    def test_partial_occlusion_no_longer_zeroes_out_every_view(self):
        # This is the actual bug report: with the OLD all-or-nothing
        # behavior, requiring every beacon ever seen to be simultaneously
        # visible in both cameras at once produced zero surviving views on
        # data like this. With per-view partial visibility and a reasonable
        # --min-points, most sample instants should now keep a view.
        tracks1, tracks2 = self._partially_occluded_tracks()
        pts1, pts2, times, ids, speeds, rejects = build_stereo_image_points(
            tracks1, tracks2, a=1.0, b=0.0, hz=10.0, max_gap_s=0.05, min_points=12,
        )
        self.assertGreater(len(pts1), 0, "expected at least some views to survive with partial visibility")
        # Every kept view must have at least min_points finite (non-NaN) points.
        for view in pts1:
            self.assertGreaterEqual(int(np.sum(~np.isnan(view[:, 0]))), 12)

    def test_kept_view_array_shape_is_dense_with_nan_gaps(self):
        # Output arrays stay dense (num_views, num_beacons, 2) - NOT ragged -
        # with NaN marking beacons absent from that particular view, so
        # downstream consumers that expect a fixed shape per view (e.g.
        # np.stack when saving the NPZ) keep working unmodified.
        tracks1, tracks2 = self._partially_occluded_tracks()
        pts1, pts2, times, ids, speeds, rejects = build_stereo_image_points(
            tracks1, tracks2, a=1.0, b=0.0, hz=10.0, max_gap_s=0.05, min_points=12,
        )
        self.assertTrue(len(pts1) > 0)
        for view1, view2 in zip(pts1, pts2):
            self.assertEqual(view1.shape, (len(ids), 2))
            self.assertEqual(view2.shape, (len(ids), 2))
            # A slot is either finite in BOTH cameras or NaN in both - never
            # finite in one but not the other (that would silently misalign
            # the two cameras' point correspondence for that beacon).
            nan1 = np.isnan(view1[:, 0])
            nan2 = np.isnan(view2[:, 0])
            np.testing.assert_array_equal(nan1, nan2)

    def test_min_points_threshold_is_respected(self):
        tracks1, tracks2 = self._partially_occluded_tracks()
        _, _, _, _, _, loose = build_stereo_image_points(
            tracks1, tracks2, a=1.0, b=0.0, hz=10.0, max_gap_s=0.05, min_points=6,
        )
        _, _, _, _, _, strict = build_stereo_image_points(
            tracks1, tracks2, a=1.0, b=0.0, hz=10.0, max_gap_s=0.05, min_points=20,
        )
        # A stricter (higher) min_points can only reject as many or more
        # candidate views than a looser one.
        self.assertGreaterEqual(strict["insufficient_points"], loose["insufficient_points"])


if __name__ == "__main__":
    unittest.main()
