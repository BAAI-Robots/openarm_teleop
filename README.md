# OpenArm Teleop

OpenArm supports 1:1 teleoperation from a leader arm to a follower arm in two control modes. See the [documentation](https://docs.openarm.dev/teleop/) for details.

## Installation
[software]https://docs.openarm.dev/software/

## Related links

- 📚 Read the [documentation](https://docs.openarm.dev/teleop/)
- 💬 Join the community on [Discord](https://discord.gg/FsZaZ4z3We)
- 📬 Contact us through <openarm@enactic.ai>

## License

Licensed under the Apache License 2.0. See [LICENSE.txt](LICENSE.txt) for details.

Copyright 2025 Enactic, Inc.

## Code of Conduct

All participation in the OpenArm project is governed by our [Code of Conduct](CODE_OF_CONDUCT.md).

## Can Bus Setup


``` bash
sudo ip link set can0 down
# configure CAN 2.0 with 1mbps
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up


sudo ip link set can1 down
# configure CAN 2.0 with 1mbps
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
```
```
ros2 action send_goal /left_joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory '{trajectory: {joint_names: ["openarm_left_joint1", "openarm_left_joint2", "openarm_left_joint3", "openarm_left_joint4", "openarm_left_joint5", "openarm_left_joint6", "openarm_left_joint7"], points: [{positions: [0.15, 0.15, 0.15, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 3, nanosec: 0}}]}}'
```