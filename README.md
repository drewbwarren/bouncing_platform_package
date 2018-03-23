# bouncing_platform_package
This is the ROS package for a juggling stewart platform robot. A high level description can be found in my [portfolio][drewb].

### Branches
Several different branches contain several different stages of development for the project.
* master - This branch contains the code for the most stable PID control attained so far.
* PID - This is the development branch for PID control. It is mostly similar to master, but some more work has been done on tuning the PID gains.
* lqr - This branch contains code for an lqr controller and actually has the most stable control of all at this point.
* feedforward - An attempt at implementing a feedforward controller together with PID was attempted on this branch, but so far it is unstable.
* state_space - A state space controller was developed on this branch, but it's control was not stable enough to continue.
* dual_camera - This branch is developing a two camera method of obtaining the ball position. It is still in development, but will likely be used for when the platform juggles the ball.

### Nodes
* [ball_tracker.py][ball] - This node ubscribes to a `sensor_msgs/Image` message on the `/usb_cam/image_raw` topic and performs some image processing to it using OpenCV. The node depends on standard ROS message libraries and the `cv2` library. The position of the ball is published on a `geometry_msgs/Point` message on the `/ball_position` topic. It should be noted that some of the image processing is color and lighting dependent and might need to be tuned to replicate, particularly the filter for the [green][grn] borders of the platform and for the [orange][orng] ball. This node opens several windows for viewing the image from the USB camera.
* [platform_controller.py][pc] - This node subscribes to the `/ball_position` topic and computes the control law for the platform. When the controls are computed they are then published in a `geometry_msgs/Point` message on the `/controls` topic. The roll is stored in the `/controls.x` value and the pitch in the `/controls.y` value. This node changes between branches. The master and PID branch instantiate [PIDControl][PID] classes for calculating the controls. Other branches have different control laws developed in this node.
* [reference.py][ref] - This node is a signal generator for providing a desired position to the controls. Currently it can provide a signal for a square wave, circular motion, and a constant input at the center of the platform.
* [serial_comm.py][ser] - This node takes the controls from the `/controls` topic and writes it to the Arduino on the platform through a serial port.
* [reset.py][res] - Running this node will move the platform back into the home position regardless of the values in the `/controls` topic.
* Non-nodes - Some files included the `/src` directory are not ROS nodes themselves, but supporting files for other nodes. PIDcontroller.py contains a class for generic PID control on a given system and params.py contains the parameters needed for control and is imported into a few other files.

### Launch Files
* [control.launch][ctrl] - This launch file starts up the usb_cam node with the appropriate paramters, the ball_tracker node, the platform_controller node, and the reference node. To be able to send commands to the platform the serial_comm node must be run separately. That node was not included in the launch file to be able to have control over when the platform is moving and when it is not.
* [ball_tracker.launch][bt] - This launches only the usb_cam node and the ball_tracker node.
* [reset.launch][reset] - This will run only the reset node to move the platform back to the home position.






[drewb]:https://drewbwarren.github.io/drewbwarren.github.io/projects/2018/03/23/juggling-robot/
[ball]:https://github.com/drewbwarren/bouncing_platform_package/blob/master/src/ball_tracker.py
[grn]:https://github.com/drewbwarren/bouncing_platform_package/blob/master/src/ball_tracker.py#L124
[orng]:https://github.com/drewbwarren/bouncing_platform_package/blob/master/src/ball_tracker.py#L53
[ref]:https://github.com/drewbwarren/bouncing_platform_package/blob/master/src/reference.py
[pc]:https://github.com/drewbwarren/bouncing_platform_package/blob/master/src/platform_controller.py
[PID]:https://github.com/drewbwarren/bouncing_platform_package/blob/master/src/PIDcontroller.py
[ser]:https://github.com/drewbwarren/bouncing_platform_package/blob/master/src/serial_comm.py
[res]:https://github.com/drewbwarren/bouncing_platform_package/blob/master/src/reset.py
[ctrl]:https://github.com/drewbwarren/bouncing_platform_package/blob/master/launch/control.launch
[bt]:https://github.com/drewbwarren/bouncing_platform_package/blob/master/launch/ball_tracker.launch
[reset]:https://github.com/drewbwarren/bouncing_platform_package/blob/master/launch/reset.launch

