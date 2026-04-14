# Scripts

These scripts are used for wheel-IMU preprocessing, time-offset estimation, and calibration result analysis.

## Files

### `rosbag_parser.py`

Use it to convert bag topics into plain text files for offline processing.

### `preprocess.py`

- trims abnormal wheel prefix data
- smooths wheel encoder signals
- interpolates IMU data onto the wheel timeline


### `estimate_time_offset.py`

Estimates wheel-to-IMU time offset from:

Method:
- fit `gyro_z ≈ a * enc1 + b * enc2`
- search the lag that gives the smallest residual

### `analysis.py`

Reads calibration logs from `Log/calib/<timestamp>` and:
- plots IMU and wheel trajectories
- plots wheel calibration states
- computes 2D ATE

### `replay_converted_bag.py`

Replays a ROS2 bag and republishes:
- IMU
- LiDAR
- wheel data converted to `liwins/msg/Wheel`

```bash
python3 scripts/replay_converted_bag.py \
  --bag ${BAG_PATH} \
  --imu-topic /livox/imu \
  --lidar-topic /livox/lidar \
  --wheel-topic /serial/wheels \
  --wheel-output-topic /serial/wheel \
  --rate 1.0
```

Use it when the bag wheel topic format does not directly match the runtime input format.
