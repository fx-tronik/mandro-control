Jak wyskoczy problem z niemożliwością załadowania bibliotek dzielonych - 

sudo nano /etc/ld.so.conf.d/ros-lib.conf

dodac linie

/opt/ros/melodic/lib

zapisac, a nastepnie

sudo ldconfig

pakiety potrzebne, a nie instalujace sie z ros-melodic-desktop-full

sudo apt install ros-melodic-soem
sudo apt install ros-melodic-rosparam-shortcut
sudo apt install ros-melodic-moveit
sudo apt install ros-melodic-trac-ik-kinematics-plugin
sudo apt install ros-melodic-trac-ik-lib
sudo apt install ros-melodic-ros-controllers
sudo apt install ros-melodic-industrial-trajectory-filters

