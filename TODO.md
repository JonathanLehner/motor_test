# TODO

- [ ] **Check the camera resolution before recording.** The stereo cameras are
  side-by-side, so the requested resolution is the *combined* frame (both eyes
  share the width). The old `640x480` default wasn't a real mode and the camera
  silently fell back to `640x240` (320x240 per eye) — too low, AprilTags came
  out ~17px and detection suffered. Run `python camera_test.py --list` to find
  the indices, then `python camera_test.py --cam-ids <ids> --no-display` to
  confirm the resolution actually sticks (watch for `<-- fell back!`). Pick a
  mode the camera advertises (`v4l2-ctl --list-formats-ext` on Linux) and pass
  it via `--width/--height` to `teleop_trigger_record.py`.
- get the camera intrinsics / calibrate stereo camera
