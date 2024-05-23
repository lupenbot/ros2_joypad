# ROS2 Joypad support package ###
## Description ##
ROS2 package which enables the use of joypads. Tested on ROS [humble](https://docs.ros.org/en/humble/index.html) distribution using a [8BitDo SN30pro+](https://support.8bitdo.com/ultimate/sn30pro_plus.html) controller.

## Configuration ##
Use the [8BitDo .yaml](./params/8bitdo_SN30ProPlus.yaml) and its [launch](./launch/joypad.launch.py) files as a reference to configure your joypad.


- Set the [`joypad_device`](./params/8bitdo_SN30ProPlus.yaml#1). Your device will be listed as a `/dev/input/jsX` device in your computer.
- If you want to publish Twist messages set the [`publish_cmd_vel`](./params/8bitdo_SN30ProPlus.yaml#) parameter to `true`. It uses as reference the `twist_linear_axis_X_tag` `twist_angular_axis_X_ta` tags.
- Remember to match node name and param file in [launcher](./launch/joypad.launch.py) file.

## Docker ##
You can test this package in a ROS2 container using the [`docker.bash`](./utils/docker.bash) script in the [utils](./utils/) folder.

```bash
# At the root of this package run
./utils/docker.bash

# use 'sudo' if you don't have the permissions
sudo ./utils/docker.bash
```

## TODO ##
This package is still under development, and in the future more features will be added. Meanwhile, have fun!

## Contact ##
Lupenbot at lupenvoleybot@gmail.com