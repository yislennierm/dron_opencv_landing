"""Draw recorded tracker stages without rerunning detection or matching."""
import cv2
import numpy as np

STAGES = ('raw', 'gray', 'features', 'reference', 'matches', 'inliers', 'alignment', 'home')


def stage_frame(simulation, stage='home'):
    if stage not in STAGES:
        raise ValueError('Unknown vision stage')
    tracker = simulation.tracker
    debug = tracker.debug_frame
    gray = debug.get('gray')
    if gray is None:
        gray = cv2.cvtColor(simulation.frame, cv2.COLOR_BGR2GRAY)
    points = debug.get('points')
    traces = debug.get('alignments', [])
    selected = debug.get('selected_reference')
    trace = next((t for t in traces if t['reference_index'] == selected), None)
    if trace is None and traces:
        trace = max(traces, key=lambda t: (t['accepted'],
                    int(t['inliers'].sum()) if t['inliers'] is not None else 0,
                    len(t['source']) if t['source'] is not None else 0))
    index = trace['reference_index'] if trace else len(tracker.references)-1 if tracker.references else None
    reference = tracker.debug_reference_images[index] if index is not None else None
    matches = len(trace['source']) if trace and trace['source'] is not None else 0
    inliers = int(trace['inliers'].sum()) if trace and trace['inliers'] is not None else 0
    observation = simulation.observation
    status = observation.reason if observation is not None else simulation.mapping
    info = dict(stage=stage, timestamp_s=debug.get('timestamp_s'), mode=debug.get('mode', 'idle'),
                features=len(points) if points is not None else 0, matches=matches, inliers=inliers,
                reference_index=index, selected_reference=selected,
                candidate_only=bool(trace and selected is None),
                alignment_status=trace['reason'] if trace else 'not_attempted',
                observation_status=status, valid=bool(observation and observation.valid))
    display = simulation.frame.copy() if stage in ('raw', 'home') else cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    title = stage.upper()
    subtitle = status
    if stage == 'gray':
        subtitle = 'Processed grayscale / rectified input' if debug.get('gray') is not None else 'Awaiting processed frame'
    elif stage == 'features':
        if points is not None:
            for point in points:
                cv2.circle(display, tuple(np.round(point).astype(int)), 3, (120, 240, 90), 1, cv2.LINE_AA)
        subtitle = f"{info['features']} selected ORB features"
    elif stage == 'reference':
        if reference is not None:
            display = cv2.cvtColor(reference, cv2.COLOR_GRAY2BGR)
            cv2.drawMarker(display, tuple(np.round(tracker.references[index].home).astype(int)),
                           (190, 245, 100), cv2.MARKER_CROSS, 24, 2)
            subtitle = f'Reference #{index+1} / {len(tracker.references)} - saved home cross'
        else:
            subtitle = 'No saved reference yet'
    elif stage in ('matches', 'inliers'):
        left = cv2.cvtColor(reference, cv2.COLOR_GRAY2BGR) if reference is not None else np.zeros_like(display)
        display = np.hstack((left, display))
        keep = trace['inliers'] if trace else None
        indices = list(range(matches)) if stage == 'matches' else list(np.flatnonzero(keep)) if keep is not None else []
        shown = indices[::max(1, int(np.ceil(len(indices)/120)))]
        for i in shown:
            a = tuple(np.round(trace['source'][i]).astype(int))
            b = tuple((np.round(trace['target'][i])+[gray.shape[1], 0]).astype(int))
            color = (140, 245, 100) if keep is not None and keep[i] else (90, 100, 240)
            cv2.line(display, a, b, color, 1, cv2.LINE_AA)
            cv2.circle(display, a, 3, color, 1)
            cv2.circle(display, b, 3, color, 1)
        subtitle = f'Ref #{index+1} -> live' if index is not None else 'No reference -> live'
        subtitle += f' | {matches} matches / {inliers} RANSAC inliers | showing {len(shown)}'
        if trace:
            subtitle += ' | '+trace['reason']
        title = 'MATCHES: GREEN INLIERS / RED OTHER' if stage == 'matches' else 'RANSAC INLIERS: BEFORE FINAL VALIDATION'
    elif stage == 'alignment':
        if trace and trace['homography'] is not None:
            h, w = gray.shape
            corners = np.float32([[[0, 0]], [[w-1, 0]], [[w-1, h-1]], [[0, h-1]]])
            corners = cv2.perspectiveTransform(corners, trace['homography'])
            if np.isfinite(corners).all():
                color = (140, 245, 100) if trace['accepted'] else (80, 170, 250)
                cv2.polylines(display, [np.int32(np.clip(corners, -10000, 10000))], True, color, 2, cv2.LINE_AA)
            if trace['inliers'] is not None:
                support = trace['target'][trace['inliers']]
                if len(support) >= 3:
                    cv2.polylines(display, [np.int32(cv2.convexHull(support))], True, (220, 170, 80), 2)
        subtitle = info['alignment_status']+' | final observation: '+status
    elif stage == 'home':
        h, w = display.shape[:2]
        center = (w//2, h//2)
        cv2.drawMarker(display, center, (160, 175, 180), cv2.MARKER_CROSS, 18, 1)
        if observation and observation.valid:
            target = tuple(round(v) for v in observation.home_px)
            cv2.arrowedLine(display, center, target, (190, 245, 100), 2, tipLength=.15)
            cv2.drawMarker(display, target, (190, 245, 100), cv2.MARKER_CROSS, 24, 2)
        subtitle = status+' | '+('confirmed home estimate' if info['valid'] else 'no confirmed home output')
    if debug.get('mode') == 'map':
        subtitle = 'MAPPING | '+subtitle
    if info['candidate_only'] and stage in ('matches', 'inliers', 'alignment'):
        subtitle = 'Candidate, not selected | '+subtitle
    # These display overlays are applied only after the tracker has finished.
    cv2.rectangle(display, (0, 0), (display.shape[1], 51), (25, 35, 42), -1)
    cv2.putText(display, title, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, .5, (215, 240, 230), 1, cv2.LINE_AA)
    cv2.putText(display, subtitle, (10, 42), cv2.FONT_HERSHEY_SIMPLEX, .4, (175, 205, 195), 1, cv2.LINE_AA)
    return display, info
