#!/bin/bash
oformat=\e[93m\e[1m\e[4m
clearformat=\e[0m
CATKIN=~/catkin_ws/
ntoinstall=0


echo This script will install dependencies needed to successfully run
echo SDRC software with ROS.
echo
echo -e "\e[93m\e[1m\e[4mDo you wish to install the following dependencies:\e[0m"

if [ ! -d "$CATKIN/src/roboteq" ]; then
printf "\troboteq_driver\n"
let "ntoinstall++"
fi
if [ ! -d "$CATKIN/src/serial" ]; then
printf "\tserial\n"
let "ntoinstall++"
fi
#printf "\n\tar_track_alvar"
if [ ! -d "$CATKIN/src/joystick_drivers" ]; then
printf "\tjoy\n"
let "ntoinstall++"
fi
if ! dpkg -l | grep -qw libspnav-dev; then
  printf "\tlibspnav-dev\n"
  let "ntoinstall++"
fi
if ! dpkg -l | grep -qw libbluetooth-dev; then
  printf "\tlibbluetooth-dev\n"
  let "ntoinstall++"
fi
if ! dpkg -l | grep -qw libcwiid-dev; then
  printf "\tlibcwiid-dev\n"
  let "ntoinstall++"
fi
if ! dpkg -l | grep -qw libusb-dev; then
  printf "\tlibusb-dev\n"
  let "ntoinstall++"
fi
if ! dpkg -l | grep -qw ros-melodic-diagnostics; then
  printf "\tros-melodic-diagnostics\n"
  let "ntoinstall++"
fi
if ! dpkg -l | grep -qw ros-melodic-roslint; then
  printf "\tros-melodic-roslint\n"
  let "ntoinstall++"
fi
if ! dpkg -l | grep -qw ros-melodic-turtlesim; then
  printf "\tros-melodic-turtlesim\n"
  let "ntoinstall++"
fi
if ! dpkg -l | grep -qw libx11-dev; then
  printf "\tlibx11-dev\n"
  let "ntoinstall++"
fi

if [ $ntoinstall == 0 ]; then
  echo -e "\e[1m\nYou are all up to date! Exiting..."
  exit 0
fi

if [ $# -eq 0 ]; then
  read -p 'y/n: ' continue
fi

if [ $1 = "-f" ] || ping -q -c 1 -W 1 8.8.8.8 >/dev/null; then
  if [ $1 = "-f" ] || [ $continue = "y" ] || [ $continue = "Y" ] || [ $continue = "yes" ] || [ $continue = "Yes" ]
  then

    echo -e "\e[93m\e[4mUpdating repository:\e[0m"
    echo apt-get update
    sudo apt-get update
    echo

    if [ ! -d "$CATKIN/src" ]; then
      echo "\n\e[93m\e[1mERROR: Cannot locate catkin workspace!"
      echo "Make sure it is set up at $CATKIN"
      exit 1
    fi

    # if [ ! -d "$CATKIN/src/roboteq" ]; then
    #   echo -e "\e[93m\e[4mInstalling roboteq_driver:\e[0m"
    #   cd $CATKIN/src/
    #   git clone https://github.com/g/roboteq.git
    #   echo
    # fi

    if [ ! -d "$CATKIN/src/serial" ]; then
      echo -e "\e[93m\e[4mInstalling serial:\e[0m"
      cd $CATKIN/src/
      git clone https://github.com/wjwwood/serial.git
      cd serial
      make
      echo
    fi

    if [ ! -d "$CATKIN/src/joystick_drivers" ]; then
      echo -e "\e[93m\e[4mInstalling joy:\e[0m"
      cd $CATKIN/src/
      git clone https://github.com/ros-drivers/joystick_drivers.git
      echo
    fi

    echo -e "\e[93m\e[4mInstalling libspnav-dev:\e[0m"
    echo apt install -y libspnav-dev
    sudo apt install -y libspnav-dev
    echo
    echo -e "\e[93m\e[4mInstalling libbluetooth-dev:\e[0m"
    echo apt install -y libbluetooth-dev
    sudo apt install -y libbluetooth-dev
    echo
    echo -e "\e[93m\e[4mInstalling libcwiid-dev:\e[0m"
    echo apt install -y libcwiid-dev
    sudo apt install -y libcwiid-dev
    echo
    echo -e "\e[93m\e[4mInstalling libusb-dev:\e[0m"
    echo apt install -y libusb-dev
    sudo apt install -y libusb-dev
    echo
    echo -e "\e[93m\e[4mInstalling ros-melodic-diagnostics:\e[0m"
    echo apt install -y ros-melodic-diagnostics
    sudo apt install -y ros-melodic-diagnostics
    echo
    echo -e "\e[93m\e[4mInstalling ros-melodic-roslint:\e[0m"
    echo apt install -y ros-melodic-roslint
    sudo apt install -y ros-melodic-roslint
    echo
    echo -e "\e[93m\e[4mInstalling ros-melodic-turtlesim:\e[0m"
    echo apt install -y ros-melodic-turtlesim
    sudo apt install -y ros-melodic-turtlesim
    echo
    echo -e "\e[93m\e[4mInstalling libx11-dev:\e[0m"
    echo apt install -y libx11-dev
    sudo apt install -y libx11-dev
    echo

    echo -e "\e[93m\e[4mRunning catkin_make:\e[0m"
    echo cd $CATKIN
    cd $CATKIN
    echo catkin_make
    catkin_make

    echo
    echo Successfully installed all dependencies!
  fi
else
  echo -e "\n\e[93m\e[1mERROR! Please check your internet connection and try again!"
fi