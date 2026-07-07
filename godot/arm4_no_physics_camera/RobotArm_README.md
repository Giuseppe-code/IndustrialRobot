# RobotArm portable

Copy these files into the target Godot project:

- `RobotArm.tscn`
- `RobotArmCameraRig.tscn`, optional wrapper with `SubViewport` and `CameraView`
- `robot_arm.gd`
- `dds.gd`
- `camera_sender.gd`, only if Python must receive camera frames
- `Assets/Joint_base.glb`
- `Assets/Joint_1.glb`
- `Assets/Joint_2.glb`
- `Assets/arm_1.glb`
- `Assets/arm_2.glb`
- `Assets/joint_3.glb`
- `Assets/joint_4.glb`
- `Assets/joint_5.glb`

Then add these autoloads in `Project > Project Settings > Globals > Autoload`:

- `DDS` -> `res://dds.gd`
- `CameraSender` -> `res://camera_sender.gd`, only if Python must receive camera frames

Instantiate `RobotArm.tscn` directly if you only need the arm in the target 3D scene.

Instantiate `RobotArmCameraRig.tscn` if you also need the camera stream used by `find_object_main.py`. This wrapper already contains a `SubViewport` and a `CameraView` texture connected to the arm script.

The arm receives DDS topics `theta0`, `theta1`, `theta2`, `theta3`, `x`, `y`, `z`, `a`, publishes `tick`, and can send a camera frame when Python publishes `read_image = 1`.

Optional inspector fields:

- `Theta 0 Box Path`, `Theta 1 Box Path`, `Theta 2 Box Path`, `Theta 3 Box Path`: optional `TextEdit` nodes for angle display.
- `Current Pose Path`: optional `Label` for the end-effector pose.
- `Camera View Path`: optional `TextureRect` whose texture is the viewport rendered by the robot camera.
- `Publish Tick`: keep enabled when Python waits on `dds.wait("tick")`.
- `Publish Camera On Request`: keep enabled when using `find_object_main.py`.
