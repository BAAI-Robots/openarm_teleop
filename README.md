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
