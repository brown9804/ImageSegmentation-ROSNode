El siguiente algoritmo funciona con nodos en ROS

Se requiere:

* Instalar ROSS indigo
* Compilar los nodos
* Descargue en instale PYUV

Por lo tanto, digite los comandos en los archivos que se encuentran en el folder "To do". Contemple que
los comandos estan para el usuario "est" por lo que debe cambiar esto por su nombre de usuario.



Si conectar su camara con el nodo falla digite lo siguiente en la consola (si manual 3 falla):

$ mkdir -p ~/catkin-ws/src
$ cd ~/catkin-ws/src
$ git clone https://github.com/bosch-ros-pkg/usb_cam.git
$ cd ..
$ catkin_make
$ source ~/catkin-ws/devel/setup.bash

roscore en una nueva terminal $ roscore

Y donde desea correr su codigo $ source 'd your usb_cam code run:

$ rosrun usb_cam usb_cam_node
