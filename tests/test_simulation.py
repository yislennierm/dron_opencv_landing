"""Closed-loop regression checks, not real flight accuracy claims."""
import unittest
from types import SimpleNamespace

import numpy as np

from landing.simulation import FlightPlan, LandingSimulation, attitude_matrix
from landing.simulation_trials import run_trial
from landing.geometry import camera_to_ground_homography, NADIR_CAMERA_TO_WORLD, project_point
from landing.terrain import ProceduralGround, ground_chunk


class SimulationTests(unittest.TestCase):
    def test_travel_lowers_leading_edge_and_tilts_thrust_toward_motion(self):
        simulation = LandingSimulation()
        simulation.phase = 'MANUAL'
        simulation.plan = FlightPlan(tilt=8.)
        for yaw in (0., np.pi/2, -np.pi/3):
            for body_velocity in ([1., 0., 0.], [-1., 0., 0.],
                                  [0., 1., 0.], [0., -1., 0.]):
                with self.subTest(yaw=yaw, velocity=body_velocity):
                    heading = attitude_matrix(0., 0., yaw)
                    velocity = heading @ body_velocity
                    simulation.attitude = np.array([0., 0., yaw])
                    simulation.velocity = velocity
                    simulation._update_attitude(1.)
                    rotation = attitude_matrix(*simulation.attitude)
                    # Independent physical checks: the leading edge is lower,
                    # and body-up (thrust) leans in the direction of travel.
                    self.assertLess((rotation @ body_velocity)[2], 0.)
                    self.assertGreater(np.dot(rotation[:2, 2], velocity[:2]), 0.)
                    np.testing.assert_allclose(simulation.rotation,
                                               rotation @ NADIR_CAMERA_TO_WORLD)

    def test_manual_takeoff_waits_for_pilot_then_brakes_on_release(self):
        simulation = LandingSimulation()
        simulation.start(dict(altitude=2.), manual=True)
        for _ in range(30):
            simulation.advance(speed=4, wind=0)
            if simulation.phase == 'MANUAL':
                break
        self.assertEqual(simulation.phase, 'MANUAL')
        height = simulation.position[2]
        for _ in range(10):
            result = simulation.advance(manual_input=[1, 0, 0], wind=0)
        self.assertEqual(result['command']['source'], 'PILOT')
        self.assertGreater(result['p'][0], .9)
        moving = float(np.linalg.norm(simulation.velocity[:2]))
        for _ in range(15):
            result = simulation.advance(wind=0)
        self.assertLess(np.linalg.norm(simulation.velocity[:2]), moving*.01)
        self.assertAlmostEqual(result['p'][2], height)
        self.assertEqual(result['command']['world_velocity_m_s'], [0., 0., 0.])

    def test_manual_climb_descend_and_return_preserve_home(self):
        simulation = LandingSimulation()
        simulation.start(dict(altitude=3.), manual=True)
        for _ in range(35):
            simulation.advance(speed=4, wind=0)
            if simulation.phase == 'MANUAL':
                break
        for _ in range(20):
            simulation.advance(manual_input=[1, -1, 0], wind=0)
        simulation.advance(manual_input=[0, 0, 1], wind=0)
        self.assertEqual(simulation.command['vertical'], 'ASCEND')
        simulation.advance(manual_input=[0, 0, -1], wind=0)
        self.assertEqual(simulation.command['vertical'], 'DESCEND')
        original = simulation.tracker.references[0]
        simulation.return_home()
        result = self.finish(simulation, wind=0)
        self.assertIs(simulation.tracker.references[0], original)
        self.assertEqual(result['phase'], 'LANDED')
        self.assertLess(result['error'], .15)

    def test_return_interrupts_takeoff_and_manual_can_override_return(self):
        simulation = LandingSimulation()
        with self.assertRaises(ValueError):
            simulation.return_home()
        simulation.start(dict(altitude=2.), manual=True)
        simulation.advance(wind=0)
        reference = simulation.tracker.references[0]
        simulation.return_home()
        self.assertEqual(simulation.phase, 'RETURN CLIMB')
        simulation.advance(manual_input=[0, -1, 0], wind=0)
        self.assertEqual(simulation.phase, 'MANUAL')
        self.assertEqual(simulation.command['source'], 'PILOT')
        simulation.return_home()
        result = self.finish(simulation, wind=0)
        self.assertEqual(result['phase'], 'LANDED')
        self.assertIs(simulation.tracker.references[0], reference)
        self.assertLess(result['error'], .15)

    def test_manual_input_validation_and_height_limit(self):
        simulation = LandingSimulation()
        simulation.start(manual=True)
        for controls in ([2, 0, 0], [0, 0], [0, float('nan'), 0]):
            with self.assertRaises(ValueError):
                simulation.advance(manual_input=controls)
        simulation.position[2] = 12.
        state = simulation.advance(manual_input=[0, 0, 1], wind=0)
        self.assertEqual(state['p'][2], 12.)
        self.assertEqual(state['command']['vertical'], 'HOLD')

    def test_command_display_uses_applied_setpoint_and_rotated_camera_axes(self):
        simulation = LandingSimulation()
        simulation.start(dict(return_yaw=90.))
        simulation.phase = 'GPS RETURN'
        simulation.position = np.array([1., 1., 4.])
        simulation.attitude[2] = np.pi/2
        observation = SimpleNamespace(valid=True, metric_valid=True,
                                      ground_offset_m=[-.2, .1], timestamp_s=.1)
        simulation.tracker.observe = lambda *args, **kwargs: observation
        simulation._step(.1, gps=0, wind=0, vision=True, occluded=False)
        command = simulation.command
        self.assertEqual(command['source'], 'VISION')
        np.testing.assert_allclose(command['world_velocity_m_s'], [-.27, .135, 0], atol=1e-10)
        # With 90-degree yaw, world -x is camera up, world +y is right.
        np.testing.assert_allclose(command['camera_horizontal_velocity_m_s'], [.135, -.27], atol=1e-10)
        np.testing.assert_allclose(simulation.velocity[:2], np.array([-.27, .135])*.3, atol=1e-10)
        self.assertEqual(command['vertical'], 'HOLD')
        self.assertEqual(command['vertical_source'], 'SIMULATOR')
        self.assertEqual(command['tracker_timestamp_s'], .1)

    def test_tracking_loss_removes_visual_command_and_holds_descent(self):
        simulation = LandingSimulation()
        simulation.start()
        simulation.phase = 'VISUAL APPROACH'
        simulation.position = np.array([.2, 0., 4.])
        observation = SimpleNamespace(valid=False, metric_valid=False, ground_offset_m=None)
        simulation.tracker.observe = lambda *args, **kwargs: observation
        simulation._step(.1, gps=1.2, wind=0, vision=True, occluded=True)
        self.assertEqual(simulation.command['source'], 'GPS')
        self.assertIsNone(simulation.command['tracker_timestamp_s'])
        self.assertEqual(simulation.command['vertical'], 'HOLD')
        self.assertGreater(simulation.command['world_velocity_m_s'][0], 0)

    def test_tilted_pixel_projection_matches_independent_ray_intersection(self):
        simulation = LandingSimulation()
        position = np.array([2., -1., 4.])
        rotation = attitude_matrix(*np.deg2rad([8., -6., 90.])) @ NADIR_CAMERA_TO_WORLD
        H = camera_to_ground_homography(simulation.camera, position, rotation)
        pixel = np.array([400., 180.])
        ray = rotation @ np.linalg.solve(simulation.camera.matrix, np.r_[pixel, 1.])
        expected = position[:2]-position[2]/ray[2]*ray[:2]
        np.testing.assert_allclose(project_point(H, pixel), expected, atol=1e-10)
        self.assertGreater(np.linalg.norm(project_point(H, [320., 240.])-position[:2]), .1)

    def test_changed_altitude_distance_heading_and_tilt_land_with_real_tracking(self):
        simulation = LandingSimulation()
        simulation.start(dict(altitude=6., distance=20., return_yaw=90., tilt=8.))
        maximum_distance = maximum_tilt = maximum_height = 0.
        for _ in range(240):
            state = simulation.advance(speed=4)
            maximum_distance = max(maximum_distance, state['distance'])
            maximum_height = max(maximum_height, state['p'][2])
            maximum_tilt = max(maximum_tilt, max(abs(v) for v in state['attitude_deg'][:2]))
            if state['phase'] == 'LANDED':
                break
        self.assertEqual(state['phase'], 'LANDED')
        self.assertGreater(maximum_distance, 19.)
        self.assertGreater(maximum_height, 5.8)
        self.assertGreater(maximum_tilt, 4.)
        self.assertAlmostEqual(state['attitude_deg'][2], 90., places=4)
        self.assertLess(state['error'], .15)
        self.assertEqual(state['metrics']['false_locks'], 0)
        self.assertEqual(state['metrics']['inaccurate_locks'], 0)

    def test_invalid_plan_does_not_reset_an_active_flight(self):
        simulation = LandingSimulation()
        simulation.start()
        for flight in ({'altitude': float('nan')}, {'distance': -1}, {'tilt': 90}, {'return_yaw': 'north'}):
            with self.assertRaises(ValueError):
                simulation.start(flight)
        self.assertEqual(simulation.phase, 'TAKEOFF')
        self.assertTrue(simulation.tracker.references)

    def test_trials_report_failed_acquisition_without_inventing_a_landing_error(self):
        simulation = LandingSimulation()
        result = run_trial(simulation, dict(altitude=2., distance=3.), obscure_return=True, max_time=20.)
        self.assertFalse(result['landed'])
        self.assertFalse(result['success'])
        self.assertIsNone(result['landing_error_m'])
        self.assertEqual(result['outcome'], 'no_return_acquisition')
        self.assertEqual(result['metrics']['false_locks'], 0)

    def test_ground_seed_reproduces_texture_without_repeated_tiles(self):
        first = ground_chunk(5, 0, 0).copy()
        other = ground_chunk(6, 0, 0)
        self.assertFalse(np.array_equal(first, other))
        ground_chunk.cache_clear()
        repeated = ground_chunk(5, 0, 0)
        np.testing.assert_array_equal(first, repeated)
        self.assertFalse(repeated.flags.writeable)
        self.assertFalse(np.array_equal(repeated, ground_chunk(5, 1, 0)))

    def test_large_world_preserves_close_up_detail_and_distant_ground(self):
        simulation = LandingSimulation()
        small = ProceduralGround(7, 32)
        large = ProceduralGround(7, 1024)
        np.testing.assert_array_equal(small.render(simulation.position, simulation.camera),
                                      large.render(simulation.position, simulation.camera))
        simulation.position = np.array([100., -100., 4.])
        frame = simulation.render()
        self.assertGreater(float(frame.std()), 10)
        self.assertFalse(np.all(frame == 65))
        self.assertLessEqual(ground_chunk.cache_info().currsize, 48)

    def finish(self, simulation, **settings):
        for _ in range(240):
            result = simulation.advance(speed=4, **settings)
            if result['phase'] == 'LANDED':
                return result
        self.fail(f"Flight did not finish: {result['phase']}")

    def test_actual_tracker_guides_landing_and_beats_gps_bias(self):
        visual = LandingSimulation()
        visual.start()
        result = self.finish(visual)
        self.assertLess(result['error'], .15)
        self.assertGreater(result['references'], 1)
        self.assertTrue(result['observation']['metric_valid'])
        self.assertEqual(result['command']['source'], 'IDLE')
        self.assertEqual(result['command']['world_velocity_m_s'], [0., 0., 0.])
        gps_only = LandingSimulation()
        gps_only.start()
        result = self.finish(gps_only, vision=False)
        self.assertGreater(result['error'], 1.)

    def test_obscured_ground_rejects_tracking_holds_descent_and_recovers(self):
        simulation = LandingSimulation()
        simulation.start()
        for _ in range(120):
            result = simulation.advance(speed=4)
            if result['phase'] == 'VISUAL APPROACH':
                break
        self.assertEqual(result['phase'], 'VISUAL APPROACH')
        height = result['p'][2]
        for _ in range(10):
            result = simulation.advance(occluded=True)
            self.assertFalse(result['observation']['valid'])
            self.assertFalse(result['observation']['metric_valid'])
            self.assertIsNone(result['observation']['ground_offset_m'])
            self.assertAlmostEqual(result['p'][2], height)
        self.assertLess(self.finish(simulation)['error'], .15)

    def test_visual_guidance_consumes_observation_instead_of_true_home(self):
        simulation = LandingSimulation()
        simulation.start()
        simulation.phase = 'GPS RETURN'
        simulation.position = np.array([1., 0., 4.])
        class Offset:
            timestamp_s = 0.1
            valid = True
            metric_valid = True
            ground_offset_m = [1., 0.]
        simulation.tracker.observe = lambda *args, **kwargs: Offset()
        simulation._step(.1, gps=0, wind=0, vision=True, occluded=False)
        # A deliberately wrong observation must steer away from true home.
        self.assertGreater(simulation.velocity[0], 0)
        self.assertEqual(simulation.velocity[2], 0)

    def test_reset_clears_reference_bank_and_observation(self):
        simulation = LandingSimulation()
        simulation.start()
        self.assertTrue(simulation.tracker.references)
        state = simulation.reset()
        self.assertEqual(state['references'], 0)
        self.assertIsNone(state['observation'])
        self.assertEqual(state['phase'], 'READY')


if __name__ == '__main__':
    unittest.main()
