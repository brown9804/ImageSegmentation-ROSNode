# ROS Node Algorithm

This algorithm works with nodes in ROS.

## Requirements

* Install ROS Indigo
* Compile the nodes
* Download and install PYUV

Therefore, type the commands in the files that are located in the "To do" folder. Note that the commands are for the user "est", so you should change this to your username.

## Troubleshooting

If connecting your camera with the node fails, type the following in the console (if manual 3 fails):

```bash
$ mkdir -p ~/catkin-ws/src
$ cd ~/catkin-ws/src
$ git clone https://github.com/bosch-ros-pkg/usb_cam.git
$ cd ..
$ catkin_make
$ source ~/catkin-ws/devel/setup.bash
```

Run `roscore` in a new terminal:

```
$ roscore
```

And where you want to run your code, source your usb_cam code run:

```
$ rosrun usb_cam usb_cam_node
```
