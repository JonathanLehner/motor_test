from teleop_trigger_record import _parse_cam_spec


def test_parse_cam_spec():
    assert _parse_cam_spec(0) == ("cv2", 0)
    assert _parse_cam_spec("2") == ("cv2", 2)
    assert _parse_cam_spec("rs") == ("rs", None)
    assert _parse_cam_spec("realsense") == ("rs", None)
    assert _parse_cam_spec("rs:012345") == ("rs", "012345")
    assert _parse_cam_spec("RealSense:ABC") == ("rs", "ABC")
    try:
        _parse_cam_spec("webcam")
    except ValueError:
        pass
    else:
        raise AssertionError("expected ValueError for bad spec")


def test_depth_png_roundtrip_is_lossless_uint16():
    # Depth is uint16 millimeters; a PNG that truncated to 8-bit would silently
    # destroy it. Verify cv2.imwrite/imread preserves values > 255.
    import tempfile, os
    import numpy as np
    import cv2
    depth = np.array([[0, 255, 256], [1000, 5000, 65535]], dtype=np.uint16)
    path = os.path.join(tempfile.gettempdir(), "depth_rt.png")
    cv2.imwrite(path, depth)
    back = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    assert back.dtype == np.uint16, back.dtype
    assert np.array_equal(back, depth)
    os.remove(path)


if __name__ == "__main__":
    test_parse_cam_spec()
    test_depth_png_roundtrip_is_lossless_uint16()
    print("ok")
