import argparse
import json
from pathlib import Path
import time

import cv2

from .benchmark import check_benchmark, run_benchmark
from .geometry import CameraModel, NADIR_CAMERA_TO_WORLD
from .replay import run_replay
from .tracker import LandingTracker


def webcam(args):
    camera = CameraModel(**json.loads(Path(args.camera).read_text())) if args.camera else None
    tracker = LandingTracker(camera=camera)
    cap = cv2.VideoCapture(args.device)
    if camera:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, camera.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, camera.height)
    else:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
    if not cap.isOpened():
        raise ValueError("Cannot open camera")
    print('SPACE: start/pause mapping | F: force reference | L: track | R: reset | Q/ESC: quit')
    mode, status, force = 'idle', 'Set home at frame center with SPACE', False
    log = open(args.log,'w') if args.log else None
    try:
        while True:
            ok, frame = cap.read()
            timestamp = time.monotonic()  # Host receipt time; sensor exposure time is not available here.
            if not ok:
                raise ValueError("Camera frame grab failed")
            gray, _ = tracker.extractor.prepare(frame)
            display = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            result = None
            if mode == 'map':
                R = NADIR_CAMERA_TO_WORLD if args.assume_level and camera else None
                status = tracker.map_frame(frame,timestamp,force=force,camera_to_world=R,
                                           telemetry_timestamp_s=timestamp if R is not None else None)
                force = False
            elif mode == 'track':
                R = NADIR_CAMERA_TO_WORLD if args.assume_level else None
                result = tracker.observe(frame,timestamp,height_m=args.height,
                    camera_to_world=R, telemetry_timestamp_s=timestamp if R is not None else None,
                    now_s=time.monotonic())
                status = f'{result.reason} | metric: {result.metric_reason}'
                if result.valid:
                    h,w = gray.shape
                    target = tuple(round(x) for x in result.home_px)
                    cv2.arrowedLine(display,(w//2,h//2),target,(0,255,0),2)
                    cv2.drawMarker(display,target,(0,255,255),cv2.MARKER_CROSS,24,2)
                    if result.metric_valid:
                        status += ' | XY(m): '+', '.join(f'{x:+.3f}' for x in result.ground_offset_m)
                if log:
                    log.write(json.dumps(result.to_dict(),allow_nan=False)+'\n')
            lines = [f'{mode.upper()} | references {len(tracker.references)}/{tracker.config.max_references}',status,
                     'SPACE map/pause | F capture | L track | R reset | Q quit']
            for i,line in enumerate(lines):
                cv2.putText(display,line,(10,22+i*22),cv2.FONT_HERSHEY_SIMPLEX,0.45,(255,255,255),1,cv2.LINE_AA)
            cv2.imshow('Natural-ground landing baseline',display)
            key = cv2.waitKey(1)&255
            if key in (27,ord('q')):
                break
            if key == ord(' '):
                mode = 'idle' if mode == 'map' else 'map'
            elif key == ord('l'):
                mode = 'track'
            elif key == ord('f'):
                mode,force = 'map',True
            elif key == ord('r'):
                tracker.reset()
                mode,status = 'idle','Reset'
    finally:
        cap.release()
        cv2.destroyAllWindows()
        if log:
            log.close()


def main():
    parser = argparse.ArgumentParser(description='Natural-ground landing research baseline')
    commands = parser.add_subparsers(dest='command',required=True)
    live = commands.add_parser('webcam')
    live.add_argument('--device',type=int,default=0)
    live.add_argument('--camera',help='Camera calibration JSON')
    live.add_argument('--height',type=float,help='Fixed current CAMERA height for controlled bench testing only')
    live.add_argument('--assume-level',action='store_true',help='Explicitly assume a level nadir camera for bench testing')
    live.add_argument('--log',help='Write observations as JSONL')
    replay = commands.add_parser('replay')
    replay.add_argument('manifest')
    replay.add_argument('--output',help='Write per-frame observations and summary JSON')
    benchmark = commands.add_parser('benchmark')
    benchmark.add_argument('--image',help='Optional texture image; default is deterministic generated texture')
    benchmark.add_argument('--output',help='Write benchmark JSON')
    benchmark.add_argument('--check',action='store_true',help='Fail if the controlled-scene regression bounds are exceeded')
    args = parser.parse_args()
    try:
        if args.command == 'webcam':
            webcam(args)
        elif args.command == 'replay':
            report = run_replay(args.manifest,args.output)
            print(json.dumps({k:v for k,v in report.items() if k != 'records'},indent=2,allow_nan=False))
        else:
            report = run_benchmark(args.image)
            if args.output:
                Path(args.output).parent.mkdir(parents=True,exist_ok=True)
                Path(args.output).write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
            if args.check:
                check_benchmark(report)
            print(json.dumps(report,indent=2,allow_nan=False))
    except ValueError as error:
        parser.exit(2,f'Error: {error}\n')


if __name__ == '__main__':
    main()
