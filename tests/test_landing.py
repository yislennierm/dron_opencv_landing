import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from landing.benchmark import rotation, texture
from landing.features import Features
from landing.geometry import CameraModel, NADIR_CAMERA_TO_WORLD as NADIR, local_scale, plane_homography, project_point
from landing.replay import run_replay, summarize
from landing.tracker import LandingTracker, Reference, TrackerConfig


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.camera = CameraModel(640,480,500,500,320,240)

    def test_translation_direction_and_inverse(self):
        H = np.array([[1,0,40],[0,1,-15],[0,0,1]],float)
        np.testing.assert_allclose(project_point(H,[220,180]),[260,165])
        np.testing.assert_allclose(project_point(np.linalg.inv(H),[260,165]),[220,180])

    def test_current_height_sets_metric_scale(self):
        np.testing.assert_allclose(self.camera.ground_offset([370,240],2,NADIR),[0.2,0])
        np.testing.assert_allclose(self.camera.ground_offset([370,240],1,NADIR),[0.1,0])

    def test_yaw_changes_world_direction(self):
        R = rotation(z=90)@NADIR
        np.testing.assert_allclose(self.camera.ground_offset([370,240],2,R),[0,0.2],atol=1e-10)

    def test_tilt_without_translation_has_zero_ground_error(self):
        R = rotation(x=10)@NADIR
        H = plane_homography(self.camera,[0,0,2],NADIR,[0,0,2],R)
        home = project_point(H,[320,240])
        self.assertGreater(np.linalg.norm(home-[320,240]),50)
        np.testing.assert_allclose(self.camera.ground_offset(home,2,R),[0,0],atol=1e-10)

    def test_ascent_scale_is_less_than_one(self):
        H = plane_homography(self.camera,[0,0,2],NADIR,[0,0,2.5],NADIR)
        self.assertAlmostEqual(local_scale(H,[320,240])[0],0.8)

    def test_nadir_home_accounts_for_initial_tilt(self):
        R = rotation(x=10)@NADIR
        home = self.camera.nadir_pixel(R)
        np.testing.assert_allclose(self.camera.ground_offset(home,2,R),[0,0],atol=1e-10)

    def test_perspective_scale_uses_local_jacobian(self):
        H = np.array([[1,0,0],[0,1,0],[0.001,0,1]],float)
        scale,_ = local_scale(H,[200,100])
        self.assertAlmostEqual(scale,1/(1.2**1.5))

    def test_resizing_preserves_ground_projection(self):
        a = self.camera.ground_offset([370,260],2,NADIR)
        b = self.camera.resized(320,240).ground_offset([185,130],2,NADIR)
        np.testing.assert_allclose(a,b)

    def test_invalid_geometry_is_rejected(self):
        for height,R in [(0,NADIR),(-1,NADIR),(2,np.eye(3)),(2,np.zeros((3,3))), (2,np.diag([1,1,-1]))]:
            with self.subTest(height=height,R=R):
                with self.assertRaises(ValueError):
                    self.camera.ground_offset([320,240],height,R)
        with self.assertRaises(ValueError):
            project_point(np.zeros((3,3)),[0,0])
        with self.assertRaises(ValueError):
            CameraModel(640,480,0,500,320,240)


class TrackingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.image = texture()
        cls.camera = CameraModel(640,480,500,500,320,240)

    def make_tracker(self, **overrides):
        config = TrackerConfig(**{'max_frame_age_s':2,**overrides})
        tracker = LandingTracker(config,self.camera)
        self.assertEqual(tracker.map_frame(self.image,0), 'captured')
        return tracker

    def test_oriented_features_recover_rotated_home(self):
        tracker = self.make_tracker(confirmation_frames=1)
        H = plane_homography(self.camera,[0,0,2],NADIR,[0.1,0.05,2.4],rotation(z=45)@NADIR)
        live = cv2.warpPerspective(self.image,H,(640,480))
        result = tracker.observe(live,0.1)
        self.assertTrue(result.valid,result.reason)
        np.testing.assert_allclose(result.home_px,project_point(H,[320,240]),atol=2)

    def test_confirmation_loss_and_recovery(self):
        tracker = self.make_tracker()
        for i in (1,2):
            result = tracker.observe(self.image,i/20)
            self.assertFalse(result.valid)
            self.assertIsNone(result.home_px)
            self.assertEqual(result.reason,'acquiring')
        self.assertTrue(tracker.observe(self.image,0.15).valid)
        lost = tracker.observe(np.full_like(self.image,120),0.2)
        self.assertFalse(lost.valid)
        self.assertIsNone(lost.home_px)
        self.assertIsNone(lost.ground_offset_m)
        self.assertFalse(tracker.observe(self.image,0.25).valid)
        self.assertFalse(tracker.observe(self.image,0.3).valid)
        self.assertTrue(tracker.observe(self.image,0.35).valid)

    def test_metric_projection_with_camera_tilt(self):
        tracker = self.make_tracker(confirmation_frames=1)
        R = rotation(x=10)@NADIR
        H = plane_homography(self.camera,[0,0,2],NADIR,[0,0,2],R)
        live = cv2.warpPerspective(self.image,H,(640,480))
        result = tracker.observe(live,0.1,height_m=2,camera_to_world=R,telemetry_timestamp_s=0.1)
        self.assertTrue(result.metric_valid,result.metric_reason)
        np.testing.assert_allclose(result.ground_offset_m,[0,0],atol=0.015)

    def test_missing_or_stale_telemetry_has_no_metric_output(self):
        tracker = self.make_tracker(confirmation_frames=1)
        result = tracker.observe(self.image,0.1)
        self.assertTrue(result.valid)
        self.assertFalse(result.metric_valid)
        result = tracker.observe(self.image,0.2,height_m=2,camera_to_world=NADIR,telemetry_timestamp_s=0)
        self.assertFalse(result.metric_valid)
        self.assertEqual(result.metric_reason,'unsynchronized_telemetry')
        self.assertIsNone(result.ground_offset_m)

    def test_stale_and_out_of_order_frames_are_invalid(self):
        tracker = self.make_tracker(confirmation_frames=1,max_frame_age_s=0.25)
        stale = tracker.observe(self.image,0.1,now_s=1)
        self.assertEqual(stale.reason,'stale_frame')
        self.assertIsNone(stale.home_px)
        self.assertEqual(tracker.observe(self.image,0.05).reason,'out_of_order_frame')

    def test_time_gap_requires_reacquisition(self):
        tracker = self.make_tracker()
        for i in range(1,4):
            tracker.observe(self.image,i/20)
        self.assertEqual(tracker.observe(self.image,1).reason,'acquiring')

    def test_unrelated_image_is_not_home(self):
        tracker = self.make_tracker(confirmation_frames=1)
        result = tracker.observe(texture(99),0.1)
        self.assertFalse(result.valid)
        self.assertIsNone(result.home_px)

    def test_reference_home_propagates_forward(self):
        tracker = self.make_tracker(confirmation_frames=1)
        H = np.array([[1,0,40],[0,1,-15],[0,0,1]],float)
        live = cv2.warpPerspective(self.image,H,(640,480))
        self.assertEqual(tracker.map_frame(live,1,force=True),'captured')
        np.testing.assert_allclose(tracker.references[-1].home,[360,225],atol=1)

    def test_initial_home_uses_synchronized_attitude(self):
        tracker = LandingTracker(camera=self.camera)
        R = rotation(x=10)@NADIR
        with self.assertRaises(ValueError):
            tracker.map_frame(self.image,0,camera_to_world=R)
        self.assertEqual(tracker.map_frame(self.image,0,camera_to_world=R,telemetry_timestamp_s=0),'captured')
        np.testing.assert_allclose(tracker.references[0].home,self.camera.nadir_pixel(R))

    def test_auto_mapping_uses_correct_scale_direction(self):
        tracker = self.make_tracker()
        H = plane_homography(self.camera,[0,0,2],NADIR,[0,0,2.8],NADIR)
        live = cv2.warpPerspective(self.image,H,(640,480))
        self.assertEqual(tracker.map_frame(live,1),'captured')
        self.assertEqual(len(tracker.references),2)

    def test_multilevel_ascent_and_descent_keep_one_home(self):
        tracker = self.make_tracker(confirmation_frames=1)
        for timestamp,height in ((1,2.6),(2,3.4)):
            H = plane_homography(self.camera,[0,0,2],NADIR,[0.04,-0.02,height],NADIR)
            frame = cv2.warpPerspective(self.image,H,(640,480))
            self.assertEqual(tracker.map_frame(frame,timestamp),'captured')
            np.testing.assert_allclose(tracker.references[-1].home,project_point(H,[320,240]),atol=1.5)
        R = rotation(z=15)@NADIR
        H = plane_homography(self.camera,[0,0,2],NADIR,[0.1,0.04,2.8],R)
        frame = cv2.warpPerspective(self.image,H,(640,480))
        result = tracker.observe(frame,3,height_m=2.8,camera_to_world=R,telemetry_timestamp_s=3)
        self.assertTrue(result.metric_valid,result.reason)
        np.testing.assert_allclose(result.ground_offset_m,[-0.1,-0.04],atol=0.02)

    def test_mapping_failure_does_not_redefine_home(self):
        tracker = self.make_tracker()
        self.assertEqual(tracker.map_frame(texture(99),1,force=True),'mapping_lost')
        self.assertEqual(len(tracker.references),1)
        np.testing.assert_array_equal(tracker.references[0].home,[320,240])

    def test_storage_is_bounded(self):
        tracker = self.make_tracker(max_references=2,max_features=200)
        self.assertEqual(tracker.map_frame(self.image,1,force=True),'captured')
        self.assertEqual(tracker.map_frame(self.image,2,force=True),'reference_limit')
        self.assertLessEqual(tracker.reference_bytes,2*(200*(2*4+32)+16))

    def test_rejects_home_outside_support(self):
        tracker = LandingTracker(camera=self.camera)
        self.assertEqual(tracker.map_frame(self.image,0,home=[0,0]),'home_outside_feature_support')

    def test_disagreeing_references_do_not_produce_a_lock(self):
        tracker = self.make_tracker(confirmation_frames=1)
        first = tracker.references[0]
        tracker.references.append(Reference(first.features,np.array([370.,240.]),0))
        self.assertFalse(tracker.observe(self.image,0.1).valid)

    def test_large_instantaneous_jump_is_rejected(self):
        tracker = self.make_tracker(confirmation_frames=1)
        self.assertTrue(tracker.observe(self.image,0.1).valid)
        H = np.array([[1,0,80],[0,1,0],[0,0,1]],float)
        live = cv2.warpPerspective(self.image,H,(640,480))
        result = tracker.observe(live,0.11)
        self.assertEqual(result.reason,'implausible_jump')
        self.assertIsNone(result.home_px)

    def test_collinear_matches_rejected(self):
        tracker = self.make_tracker()
        points = np.float32([[x,240] for x in range(40,600,10)])
        desc = np.random.default_rng(5).integers(0,256,(len(points),32),dtype=np.uint8)
        features = Features(points,desc,(480,640))
        self.assertIsNone(tracker._align(Reference(features,np.array([320.,240.]),0),features))

    def test_bad_frame_and_calibration_size_rejected(self):
        tracker = self.make_tracker()
        with self.assertRaises(ValueError):
            tracker.observe(np.zeros((240,320),np.uint8),0.1)
        with self.assertRaises(ValueError):
            tracker.observe(np.zeros((480,640),np.float32),0.1)

    def test_distortion_is_removed_before_matching(self):
        camera = CameraModel(640,480,500,500,320,240,(-0.1,0.01,0,0,0))
        tracker = LandingTracker(camera=camera)
        processed,rectified = tracker.extractor.prepare(self.image)
        expected = cv2.undistort(self.image,camera.matrix,np.array(camera.distortion))
        self.assertLess(float(np.mean(np.abs(processed.astype(float)-expected))),0.1)
        self.assertEqual(rectified.distortion,())


class ReplayTests(unittest.TestCase):
    def test_video_replay(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            video = directory/'ground.avi'
            writer = cv2.VideoWriter(str(video),cv2.VideoWriter_fourcc(*'MJPG'),20,(640,480))
            self.assertTrue(writer.isOpened())
            image = cv2.cvtColor(texture(),cv2.COLOR_GRAY2BGR)
            for _ in range(5):
                writer.write(image)
            writer.release()
            frames = [dict(frame_index=i,timestamp_s=i/20,mode='map' if i == 0 else 'track') for i in range(5)]
            manifest = directory/'video.json'
            manifest.write_text(json.dumps(dict(video='ground.avi',config=dict(max_frame_age_s=2),frames=frames)))
            report = run_replay(manifest)
            self.assertEqual(report['summary']['valid_frames'],2)

    def test_replay_rejects_nonmonotonic_timestamps(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            cv2.imwrite(str(directory/'ground.png'),texture())
            frame = dict(image='ground.png',timestamp_s=0,mode='map')
            path = directory/'bad.json'
            path.write_text(json.dumps(dict(frames=[frame,frame])))
            with self.assertRaises(ValueError):
                run_replay(path)

    def test_file_replay_and_report(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            cv2.imwrite(str(directory/'ground.png'),texture())
            frames = [dict(image='ground.png',timestamp_s=0,mode='map')]
            frames += [dict(image='ground.png',timestamp_s=i/20,expected_visible=True,
                            ground_truth_home_px=[320,240]) for i in range(1,5)]
            manifest = directory/'replay.json'
            manifest.write_text(json.dumps(dict(config=dict(max_frame_age_s=2),frames=frames)))
            result = run_replay(manifest,directory/'report.json')
            self.assertEqual(result['summary']['valid_frames'],2)
            self.assertLess(result['summary']['home_error_px']['maximum'],1)
            self.assertEqual(result['summary']['recovery_frames'],[3])
            self.assertEqual(json.loads((directory/'report.json').read_text())['reference_count'],1)

    def test_negative_labels_and_inaccurate_locks_are_separate(self):
        records = [dict(timestamp_s=0,valid=True,metric_valid=False,latency_ms=1,
                        expected_visible=False),
                   dict(timestamp_s=1,valid=True,metric_valid=False,latency_ms=1,
                        expected_visible=True,home_px=[100,100],ground_truth_home_px=[0,0])]
        summary = summarize(records)
        self.assertEqual(summary['false_locks'],1)
        self.assertEqual(summary['inaccurate_locks'],1)


if __name__ == '__main__':
    unittest.main()
