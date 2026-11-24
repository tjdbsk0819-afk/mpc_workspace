"""Exercise real port availability and immediate WebRTC restart semantics."""
import errno
import socket
import unittest
from unittest.mock import patch

from simulation_runtime.environment import _check_port_available, WebRtcEnvironmentError


class WebRtcPortTests(unittest.TestCase):
    def test_free_tcp_port_is_reserved_until_probe_closes(self):
        with _check_port_available(0, socket.SOCK_STREAM, 'signal TCP') as probe:
            port = probe.getsockname()[1]
            with self.assertRaises(WebRtcEnvironmentError):
                _check_port_available(port, socket.SOCK_STREAM, 'signal TCP')

    def test_active_tcp_server_with_reuseaddr_is_rejected(self):
        with socket.socket() as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(('127.0.0.1', 0))
            server.listen(1)
            with self.assertRaises(WebRtcEnvironmentError) as raised:
                _check_port_available(server.getsockname()[1], socket.SOCK_STREAM, 'signal TCP')
            self.assertEqual(raised.exception.__cause__.errno, errno.EADDRINUSE)

    def test_stopped_tcp_server_in_time_wait_is_reusable(self):
        with socket.socket() as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(('127.0.0.1', 0))
            server.listen(1)
            port = server.getsockname()[1]
            with socket.create_connection(('127.0.0.1', port), timeout=2) as client:
                accepted, _ = server.accept()
                with accepted:
                    # Server sends the first FIN, so TIME_WAIT remains on its port.
                    accepted.shutdown(socket.SHUT_WR)
                    self.assertEqual(client.recv(1), b'')
        with socket.socket() as old_probe:
            with self.assertRaises(OSError) as raised:
                old_probe.bind(('0.0.0.0', port))
            self.assertEqual(raised.exception.errno, errno.EADDRINUSE)
        with _check_port_available(port, socket.SOCK_STREAM, 'signal TCP'):
            pass

    def test_active_udp_server_is_rejected(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server:
            server.bind(('127.0.0.1', 0))
            with self.assertRaises(WebRtcEnvironmentError):
                _check_port_available(server.getsockname()[1], socket.SOCK_DGRAM, 'stream UDP')

    def test_permission_error_is_not_reported_as_another_isaac(self):
        with patch('simulation_runtime.environment.socket.socket') as constructor:
            probe = constructor.return_value
            probe.bind.side_effect = OSError(errno.EACCES, 'Permission denied')
            with self.assertRaisesRegex(WebRtcEnvironmentError, 'Permission denied') as raised:
                _check_port_available(49100, socket.SOCK_STREAM, 'signal TCP')
            self.assertNotIn('Another server', str(raised.exception))
            probe.close.assert_called_once()


if __name__ == '__main__':
    unittest.main()
