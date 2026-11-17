"""Local HTTP access protects calibration actions and serves only built assets."""

from http.client import HTTPConnection
from pathlib import Path
from threading import Thread

from so101_setup.server import (
    CalibrationApplication,
    CalibrationHTTPServer,
    request_handler,
)


def test_api_requires_session_authorization_and_static_requests_stay_inside_assets(
    tmp_path: Path,
) -> None:
    assets = tmp_path / "assets"
    assets.mkdir()
    (assets / "index.html").write_text("Calibration UI")
    (tmp_path / "private.txt").write_text("Private config")
    application = CalibrationApplication(tmp_path, tmp_path / "calibration")
    with CalibrationHTTPServer(
        ("127.0.0.1", 0), request_handler(application, assets, "session-token")
    ) as server:
        thread = Thread(target=server.serve_forever, kwargs={"poll_interval": 0.01})
        thread.start()
        try:
            connection = HTTPConnection("127.0.0.1", server.server_port, timeout=5)
            for method, path, body, expected in [
                ("GET", "/api/state", None, 401),
                ("POST", "/api/action", '{"action":"capture_reference"}', 403),
                ("GET", "/../private.txt", None, 404),
                ("GET", "/", None, 200),
            ]:
                connection.request(method, path, body)
                response = connection.getresponse()
                assert response.status == expected
                if path == "/":
                    cookie_header = response.getheader("Set-Cookie")
                    assert cookie_header is not None
                    session_cookie = cookie_header.split(";", 1)[0]
                assert b"Private config" not in response.read()
            connection.request("GET", "/api/state", headers={"Cookie": session_cookie})
            response = connection.getresponse()
            # USB access may be busy when the real calibrator is running.
            assert response.status in (200, 503)
            response.read()
            connection.request(
                "POST",
                "/api/action",
                '{"action":"restart"}',
                {"Cookie": session_cookie, "Origin": "https://evil.example"},
            )
            response = connection.getresponse()
            assert response.status == 403
            response.read()
            # An authenticated client still cannot invent a powered-motion action.
            connection.request(
                "POST",
                "/api/action",
                '{"action":"enable_torque"}',
                {"Cookie": session_cookie, "Origin": f"http://127.0.0.1:{server.server_port}"},
            )
            response = connection.getresponse()
            assert response.status == 400
            response.read()
            connection.close()
        finally:
            server.shutdown()
            thread.join(timeout=5)
