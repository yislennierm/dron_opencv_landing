"""Local browser UI and closed-loop OpenCV simulation API."""
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import base64
import cv2
from pathlib import Path
import threading
import webbrowser

from .simulation import LandingSimulation
from .vision_debug import STAGES, stage_frame


def make_handler(image=None, seed=7, ground_size=256.):
    sessions = {}
    lock = threading.Lock()

    class Handler(SimpleHTTPRequestHandler):
        def do_POST(self):
            if self.path != '/api/simulation':
                self.send_error(404)
                return
            try:
                length = int(self.headers.get('Content-Length', 0))
                if not 0 < length <= 4096:
                    raise ValueError('Invalid request size')
                data = json.loads(self.rfile.read(length))
                session = data['session']
                if not isinstance(session, str) or not 1 <= len(session) <= 80:
                    raise ValueError('Invalid session')
                action = data.get('action', 'step')
                if action not in ('reset', 'start', 'step', 'return', 'pilot', 'view'):
                    raise ValueError('Unknown action')
                stage = data.get('view', 'home')
                if stage not in STAGES:
                    raise ValueError('Unknown vision stage')
                with lock:
                    if session not in sessions:
                        if len(sessions) >= 16:
                            raise ValueError('Session limit reached; restart the simulator server')
                        sessions[session] = LandingSimulation(image, seed=seed, ground_size=ground_size)
                    simulation = sessions[session]
                    if action == 'reset':
                        result = simulation.reset()
                    elif action == 'start':
                        result = simulation.start(data.get('flight'), manual=data.get('manual', False))
                    elif action == 'return':
                        result = simulation.return_home()
                    elif action == 'pilot':
                        result = simulation.take_control()
                    elif action == 'view':
                        result = simulation.snapshot()
                    else:
                        result = simulation.advance(**data.get('settings', {}))
                    display, result['vision_debug'] = stage_frame(simulation, stage)
                    ok, encoded = cv2.imencode('.jpg', display, [cv2.IMWRITE_JPEG_QUALITY, 78])
                    if not ok:
                        raise ValueError('Cannot encode vision view')
                    result['camera'] = base64.b64encode(encoded).decode('ascii')
                payload = json.dumps(result, allow_nan=False).encode()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)
            except (ValueError, TypeError, KeyError) as error:
                self.send_error(400, str(error))

        def log_message(self, format, *args):
            # Avoid a terminal line for every camera frame.
            if args and str(args[1] if len(args) > 1 else '').startswith(('4', '5')):
                super().log_message(format, *args)

    return partial(Handler, directory=str(Path(__file__).parent / 'sim'))


def serve(port=8765, open_browser=True, image=None, seed=7, ground_size=256.):
    # Generate/validate the terrain before accepting sessions.
    LandingSimulation(image, seed=seed, ground_size=ground_size)
    with ThreadingHTTPServer(('127.0.0.1', port), make_handler(image, seed=seed, ground_size=ground_size)) as server:
        url = f'http://127.0.0.1:{server.server_port}'
        print(f'Landing simulator: {url}\nCtrl+C to stop.', flush=True)
        if open_browser:
            webbrowser.open(url)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
