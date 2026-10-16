```bash
cd ~/ECElabs/lab_files/
```

```bash
code .
```
```bash
source ur3_setup.sh
```
```bash
ros2 launch ur_robot_driver ur_control.launch.py \
ur_type:=ur3e robot_ip:=192.168.10.53 \
headless_mode:=true launch_rviz:=true \
kinematics_params_file:="Robot2_calibration.yaml"
```

```bash
ros2 run ece470labs lab4_exec points
```
