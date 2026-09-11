# uorb_demo

uORB bridge example and test fixture (`CONFIG_BOARD_CM4_APP="uorb_demo"`,
the `rpmsg` board variant). Links the bridge layer `lib/uorb_lite.c` and
its fault kinds `lib/uorb_fault.c` through `app.mk`; topic structs come from
the generated headers in the PX4 build directory, so both cores share one
layout.

- Loopback for `rpmsg_uorb test`: every `debug_value` from the CM7 comes
  back as `debug_key_value` with key `cm4` and the same value.
- `gyro_analyser.c`: every `sensor_gyro` sample of every IMU, per-instance
  mean, mean-removed RMS and peak, zero-crossing frequency estimate, rate
  and largest sample gap, published once a second per IMU as `debug_array`
  "cm4gyro" (layout in `gyro_analyser.h`). Hard-float.

```
nsh> rpmsg_uorb start                     # CM7 side of the bridge (autostarted)
nsh> rpmsg_uorb test 100                  # debug_value -> CM4 -> debug_key_value round trips
nsh> rpmsg_uorb status
nsh> listener debug_array 3               # one line per IMU slot
nsh> rpmsg fault list                     # runtime kinds plus runt badtype badtopic badlen badsize unknown flood
```
