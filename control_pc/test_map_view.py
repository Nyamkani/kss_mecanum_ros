"""Headless map decoding/geometry regression tests (no ROS or display required)."""
import base64
import math
from types import SimpleNamespace
import unittest
import zlib

from map_view import MapView, decode_map


class MapTests(unittest.TestCase):
    def packet(self, data=bytes([255, 0, 100, 50])):
        return dict(type='map', sequence=1, width=2, height=2, resolution=0.5,
                    origin=dict(x=10., y=20., qz=math.sin(math.pi/4), qw=math.cos(math.pi/4)),
                    encoding='zlib+base64:int8', data=base64.b64encode(zlib.compress(data)).decode())

    def test_round_trip(self):
        decoded = decode_map(self.packet())
        self.assertEqual(decoded['cells'], bytes([255, 0, 100, 50]))
        self.assertEqual(decoded['origin']['x'], 10.)
        self.assertNotIn('data', decoded)

    def test_limits_and_corrupt_data(self):
        for data in (b'', b'\x00'*3, b'\x00'*5, b'\x00'*100000, bytes([101]*4)):
            with self.subTest(data_length=len(data)), self.assertRaises(ValueError):
                decode_map(self.packet(data))
        for changes in ({'width': 0}, {'height': 1048577}, {'resolution': float('nan')},
                        {'encoding': 'raw'}, {'data': '!!!'}, {'sequence': -1}):
            packet = self.packet(); packet.update(changes)
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                decode_map(packet)
        packet = self.packet()
        packet['data'] = base64.b64encode(zlib.compress(b'\x00'*4) + b'extra').decode()
        with self.assertRaises(ValueError):
            decode_map(packet)

    def test_origin_rotation_and_screen_y(self):
        view = SimpleNamespace(map=decode_map(self.packet()), offset_x=5., offset_y=7.,
                               scale_x=10., scale_y=10.)
        # Local cell-center (0.25, 0.25) -> world (9.75, 20.25) at origin yaw pi/2.
        x, y = MapView.world_to_canvas(view, 9.75, 20.25)
        self.assertAlmostEqual(x, 10.)
        self.assertAlmostEqual(y, 22.)
        x2, y2 = MapView.world_to_canvas(view, 9.75, 20.75)
        self.assertGreater(x2, x)
        self.assertAlmostEqual(y2, y)

    def test_pixel_world_round_trip(self):
        for yaw in (0.0, 0.73, -1.4, math.pi / 2):
            packet = self.packet()
            packet['origin'].update(qz=math.sin(yaw / 2), qw=math.cos(yaw / 2))
            view = SimpleNamespace(map=decode_map(packet), offset_x=37., offset_y=19.,
                                   scale_x=2.3, scale_y=1.7)
            for pixel in ((37., 19.), (39., 21.), (41.6, 22.4)):
                world = MapView.canvas_to_world(view, *pixel)
                actual = MapView.world_to_canvas(view, *world)
                for a, b in zip(actual, pixel):
                    self.assertAlmostEqual(a, b)


if __name__ == '__main__':
    unittest.main()
