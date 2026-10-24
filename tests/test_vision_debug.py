"""Diagnostics must show the real processing pass without changing its output."""
import unittest
from unittest.mock import patch

import numpy as np

from landing.benchmark import texture
from landing.tracker import LandingTracker
from landing.simulation import LandingSimulation
from landing.vision_debug import STAGES, stage_frame


class VisionDebugTests(unittest.TestCase):
    def test_opt_in_trace_matches_real_selected_alignment_and_preserves_results(self):
        frame = texture()
        plain, debug = LandingTracker(), LandingTracker(debug=True)
        for tracker in (plain, debug):
            self.assertEqual(tracker.map_frame(frame, 0), 'captured')
        for i in range(1, 5):
            expected = plain.observe(frame, i*.1)
            actual = debug.observe(frame, i*.1)
            self.assertEqual(actual.valid, expected.valid)
            self.assertEqual(actual.reason, expected.reason)
            if actual.valid:
                np.testing.assert_allclose(actual.home_px, expected.home_px, atol=.01)
        trace = next(t for t in debug.debug_frame['alignments'] if t['reference_index'] == actual.reference_index)
        self.assertTrue(trace['accepted'])
        self.assertEqual(len(trace['source']), actual.quality['matches'])
        self.assertEqual(int(trace['inliers'].sum()), actual.quality['inliers'])
        np.testing.assert_array_equal(debug.debug_reference_images[0], frame)
        self.assertEqual(plain.debug_frame, {})
        self.assertEqual(plain.debug_reference_images, [])
        debug.reset()
        self.assertEqual(debug.debug_frame, {})
        self.assertEqual(debug.debug_reference_images, [])

    def test_stage_views_do_not_rerun_vision_or_modify_the_frame(self):
        simulation = LandingSimulation()
        simulation.start()
        for i in range(1, 5):
            simulation.observation = simulation.tracker.observe(simulation.frame, i*.1,
                height_m=.5, camera_to_world=simulation.rotation, telemetry_timestamp_s=i*.1)
        self.assertTrue(simulation.observation.valid)
        original = simulation.frame.copy()
        position = simulation.position.copy()
        before = simulation.tracker.reference_bytes
        with patch.object(simulation.tracker.extractor, 'extract', side_effect=AssertionError('Recomputed features')):
            for stage in STAGES:
                display, info = stage_frame(simulation, stage)
                self.assertEqual(display.dtype, np.uint8)
                self.assertEqual(display.shape, (480, 1280 if stage in ('matches', 'inliers') else 640, 3))
                self.assertEqual(info['stage'], stage)
                self.assertEqual(info['matches'], simulation.observation.quality['matches'])
                self.assertEqual(info['inliers'], simulation.observation.quality['inliers'])
                self.assertEqual(info['timestamp_s'], .4)
                self.assertFalse(info['candidate_only'])
        np.testing.assert_array_equal(simulation.frame, original)
        np.testing.assert_array_equal(simulation.position, position)
        self.assertEqual(simulation.tracker.reference_bytes, before)

    def test_lost_tracking_clears_previous_matches_and_confirmed_home(self):
        simulation = LandingSimulation()
        simulation.start()
        simulation.tracker.observe(simulation.frame, .1)
        simulation.frame = np.full_like(simulation.frame, 65)
        simulation.observation = simulation.tracker.observe(simulation.frame, .2)
        self.assertFalse(simulation.observation.valid)
        for stage in ('features', 'matches', 'inliers', 'alignment', 'home'):
            _, info = stage_frame(simulation, stage)
            self.assertEqual(info['features'], 0)
            self.assertEqual(info['matches'], 0)
            self.assertEqual(info['inliers'], 0)
            self.assertFalse(info['valid'])
            self.assertEqual(info['observation_status'], 'no_good_alignment')

    def test_initial_views_and_invalid_stage(self):
        simulation = LandingSimulation()
        for stage in STAGES:
            _, info = stage_frame(simulation, stage)
            self.assertEqual(info['mode'], 'idle')
            self.assertEqual(info['features'], 0)
            self.assertIsNone(info['reference_index'])
        with self.assertRaises(ValueError):
            stage_frame(simulation, 'unknown')


if __name__ == '__main__':
    unittest.main()
