import io
import struct
import unittest

from paxini_sdk import S1813_ELITE_COORDINATES
from paxini_sdk.s1813 import (
    build_read_request,
    calculate_lrc,
    decode_force,
    parse_read_response,
    receive_response,
)


def make_response(address: int, start: int, data: bytes, status: int = 0) -> bytes:
    body = struct.pack("<BBBIHB", address, 0, 0xFB, start, len(data), status) + data
    frame = b"\xAA\x55" + struct.pack("<H", len(body)) + body
    return frame + bytes((calculate_lrc(frame),))


class ProtocolTests(unittest.TestCase):
    def test_known_request_frames(self) -> None:
        self.assertEqual(
            build_read_request(1, 1008, 3),
            bytes.fromhex("55 aa 09 00 01 00 fb f0 03 00 00 03 00 06"),
        )
        self.assertEqual(
            build_read_request(1, 1038, 93),
            bytes.fromhex("55 aa 09 00 01 00 fb 0e 04 00 00 5d 00 8d"),
        )

    def test_response_sync_and_payload_offset(self) -> None:
        payload = bytes(range(93))
        response = make_response(3, 1038, payload, status=0xA5)
        received = receive_response(io.BytesIO(b"noise" + response))
        self.assertEqual(received, response)
        self.assertEqual(parse_read_response(response, 3, 1038, 93), payload)

    def test_force_signedness_and_scale(self) -> None:
        force = decode_force(bytes.fromhex("ff 80 fa"))
        self.assertEqual((force.fx, force.fy, force.fz), (-0.1, -12.8, 25.0))

    def test_coordinate_count_and_order(self) -> None:
        self.assertEqual(len(S1813_ELITE_COORDINATES), 31)
        self.assertEqual(
            tuple(point.index for point in S1813_ELITE_COORDINATES), tuple(range(1, 32))
        )


if __name__ == "__main__":
    unittest.main()
