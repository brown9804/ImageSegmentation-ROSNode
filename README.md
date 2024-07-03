# ROS Node Image Segmentation Project

Costa Rica

Belinda Brown, belindabrownr04@gmail.com

[![GitHub](https://img.shields.io/badge/--181717?logo=github&logoColor=ffffff)](https://github.com/)
[brown9804](https://github.com/brown9804)

----------

This project aims to develop a ROS (Robot Operating System) node that performs image segmentation on input images obtained from a camera sensor. Image segmentation is a computer vision technique that partitions an image into multiple segments or regions to simplify its representation and make it easier to analyze. The project will involve developing algorithms for segmenting images into meaningful regions based on color, intensity, texture, or other visual attributes.

<img width="700" src="https://intorobotics.com/wp-content/uploads/2023/09/template-for-a-ros-subscriber-in-python.jpg" alt="rosImg"/> 

[reference](https://intorobotics.com/template-for-a-ros-subscriber-in-python/)

## Key Components
1. ROS Node Development: The project will involve creating a ROS node that subscribes to image messages from a camera sensor and processes them using image segmentation algorithms.
2. Image Segmentation Algorithms: The project will explore various image segmentation algorithms such as thresholding, region-based segmentation, edge detection, and clustering methods to partition images into distinct regions.
3. Parameter Tuning: Fine-tuning the parameters of the segmentation algorithms to achieve accurate and reliable segmentation results, considering factors such as lighting conditions, noise, and image resolution.
4. Visualization: Implementing visualization tools within the ROS environment to display segmented images and visualize the segmentation process for debugging and analysis.
5. Integration: Integrating the image segmentation node into a larger robotic system or application to demonstrate its functionality in real-world scenarios.

## Benefits

Overall, the ROS Node Image Segmentation Project aims to leverage the capabilities of ROS and computer vision techniques to enhance the perception and decision-making abilities of robots through efficient image segmentation processes.

- Improved Image Analysis: Image segmentation helps in extracting meaningful information from images, enabling robots to better understand their surroundings and make informed decisions.
- Enhanced Robotic Perception: By segmenting images into relevant regions, robots can recognize objects, obstacles, and other elements in their environment more effectively.
- Versatile Applications: The developed ROS node can be applied to various robotic tasks such as object recognition, navigation, manipulation, and surveillance.


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

## Problem statement 

Design, programming in the C programming language and testing a project in `CodeBlocks` called `image_segmentation_project` that performs:

1. Read a color image in “.bmp” format from the hard disk.
2. Calculate the intensity image of the color image obtained in point a) and store the resulting intensity image on the hard disk under the name "IntensityImage.bmp", in ".bmp" format.
3.  Threshold the intensity image obtained in point b) and store the resulting segmented image on the hard disk under the name “segmented image.bmp”, in “.bmp” format. The threshold should be calculated automatically by applying the Kittler algorithm to the intensity image. The optimal threshold, weights, means, and variances should be displayed on the terminal and saved in a text file called “optimalResultsSegunKittler.txt”. For the tests use the image "table1_00.bmp", which can be downloaded from Virtual Mediation, for which the Kittler algorithm gives the following optimal values:


```
Ubuntu 32 bit:

th=165

mean1=148.451151

var1=15.428531

mean2=218.506530

var2=10.100316


Ubuntu 64 bit:

th=166

mean1=149.287913 

var1=15.381974 

mean2=219.336596 

var2=9.867214
```

The directory and name of the input color image and its dimensions (width and height) must be read from a control parameter file called “current_control_parameters.txt”. Likewise, in said file the output directory must be indicated, where the images "imagenDeIntensida.bmp" and "imagenSegmentada.bmp", and the text file "ResultsOptimosSegunKittler.txt", must be stored. An example of a control parameter file is as follows:


```
Entrada: input/cuadro1_00.bmp 

ancho: 756

alto: 455

Salida: output/
```

![](https://github.com/brown9804/Image_Segmentation_Project/blob/master/docs/img/output_img_seg_project_def.png)

## How to run

To run the program is to go to the folder through your console or terminal to the folder in which the program is located.

You must enter and type in the console or terminal:

```
$ make
```

