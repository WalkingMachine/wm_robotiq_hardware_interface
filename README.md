# wm_robotiq_hardware_interface

Package pour le hardware interface des moteurs robotiq de sara.
Ce package agit comme un plugin pour sara_control. Par conséquent, aucun launch direct n'est néssésaire.
### Instalation
```sh
git clone https://github.com/WalkingMachine/sara_launch.git
git clone https://github.com/WalkingMachine/sara_control.git
git clone https://github.com/WalkingMachine/sara_description.git
git clone https://github.com/WalkingMachine/robotiq_140_description.git
sudo apt-get install ros-kinetic-ros-control
catkin_make
```
### Pour les tests manuels
```
roslaunch sara_description sara_description
roslaunch sara_control sara_control
roslaunch sara_launch sara_controllers
```  
