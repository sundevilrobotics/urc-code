#!/bin/bash

test_mode=False

Version()
{
  echo "install.sh - Sun Devil Robotics Club urc-code Dependencies Installer"
  echo "Version 1.0"
  echo "Created by Brandon Rice"
  echo "Copyright 2021"
  echo
}

Help()
{
  # Display Help
  Version
  echo "Syntax: install.sh [-h|t|v|w]"
  echo "Options:"
  echo "h     Print this Help."
  echo "t     Enable Test Mode (answer y to all questions)."
  echo "v     Print the version information."
  echo "w     Go to the SDRC urc-code website."
  echo
}

while getopts ":htwv" option; do
   case $option in
      h) # display Help
        Help
        exit;;
      t) # test mode for automated gtests
        test_mode=True
        echo "TEST MODE ENABLED";;
      v)
        Version
        exit;;
      w)
        echo "Opening website: https://github.com/sundevilrobotics/urc-code.git"
        echo
        python3 -m webbrowser https://github.com/sundevilrobotics/urc-code.git
        exit;;
      *)
        echo "Incorrect arguments passed!"
        echo ""
        Help
        exit;;
   esac
done

format='\e[93m\e[1m\e[4m'
yellow='\e[93m'
cyan='\e[96m'
bold='\e[1m'
clearformat='\e[0m'
underline='\e[4m'
CATKIN=~/catkin_ws/
ntoinstall=0

# clear
echo -e "\e[?1049h"

display_center(){
    columns="$(tput cols)"
    while IFS= read -r line; do
        printf "%*s\n" $(( (${#line} + columns) / 2)) "$line"
    done < "$1"
}

prog() {
  exec < /dev/tty
  oldstty=$(stty -g)
  stty raw -echo min 0
  # on my system, the following line can be replaced by the line below it
  echo -en "\033[6n" > /dev/tty
  # tput u7 > /dev/tty    # when TERM=xterm (and relatives)
  IFS=';' read -r -d R -a pos
  stty $oldstty
  # change from one-based to zero based so they work with: tput cup $row $col
  row=$((${pos[0]:2} - 1))    # strip off the esc-[
  col=$((${pos[1]} - 1))

  tput cup $(tput lines) 0 

    local w=80 p=$1;  shift
    # create a string of spaces, then change them to dots
    printf -v dots "%*s" "$(( $p*$w/100 ))" ""; dots=${dots// /.};
    # print those dots on a fixed-width space plus the percentage etc. 
    printf "\r\e[K|%-*s| %3d %% %s" "$w" "$dots" "$p" "$*"; 
    
  row=$(($row + 1))
  echo "\033[${row};${col}H"
}

# IFS=$'\n'
# for word in $(< logo.txt)
# do
#     print_center $word
# done



display_center "logo.txt"

# ./prog.sh &


echo ""
echo ""

echo -e "${bold}Sun Devil Robotics Club urc-code Dependencies Installer${clearformat}"
echo ""
echo This script will install dependencies needed to successfully run the sdrc_urc ROS package!
echo ""
echo You are on ROS $ROS_DISTRO.
echo -e "${format}Do you wish to continue?${clearformat}"

if [ $test_mode == "False" ]; then
read -p 'y/n: ' cont
case $cont in  
  y|Y) cont=True ;; 
  n|N) cont=False ;; 
  *) cont=False ;; 
esac
else
  cont=True
  echo "y"
fi

if [ $cont == "False" ]; then
  echo "Exiting."
  echo -e "\e[?1049l"
  exit 0
fi
echo ""

# tput cup $(tput lines) 0 
# for x in {1..100} ; do
#     prog "$x" still working...
#     sleep .1   # do some work here
# done ; echo

echo -e "${cyan}${underline}Checking if rosdep is installed.${clearformat}"
if [ $ROS_DISTRO == melodic ]; then
  pkg=python-rosdep
else
  pkg=python3-rosdep
fi

if dpkg --get-selections | grep -q "^$pkg[[:space:]]*install$" >/dev/null; then
  echo "rosdep is installed."
else
  echo "rosdep is not installed. Installing rosdep..."
  sudo apt install -y $pkg
fi
echo ""

echo "The following sdrc_urc packages were found:"
for d in */ ; do
  if [[ $d == sdrc_urc_* ]]; then
    pkg=${d::-1}
    echo "$pkg"
  fi
done
echo ""

echo -e "${cyan}${underline}Checking dependencies for each package using rosdep.${clearformat}"
echo -e "${bold}Any \"ERROR: Cannot locate rosdep definition\" should be fixed later when additional packages are being installed from source.${clearformat}"
for d in */ ; do
  if [[ $d == sdrc_urc_* ]]; then
    pkg=${d::-1}
    echo -e "${yellow}$pkg${clearformat}"
    rosdep install $pkg #-r
    # echo "apt,$pkg," >> packages_to_uninstall.csv
  fi
done
echo ""

# echo -e "${cyan}${underline}Installing additional dependencies from source (that rosdep cannot handle)...${clearformat}"
# while IFS=, read -r name link
# do
#   wd=$(pwd)
#   cd ..

#   if [ -d "./$name" ]; then
#     echo -e "${yellow}$name${clearformat} is installed. Checking if it needs to be updated..."
#     cd ./$name
#     git pull
#     cd $wd
#   else
#     echo -e "${yellow}$name${clearformat} is not installed. Installing $name:"
#     $link
#     cd $wd
#     echo "source,$name," >> packages_to_uninstall.csv
#   fi

# done < packages.csv
# echo ""

echo -e "${cyan}${underline}Checking Gazebo version and updating if necessary...${clearformat}"
gzversion=$(gazebo --version | head -n 1)
gzversion="Gazebo multi-robot simulator, version 9.19.0"
# echo $gzversion
gzversionmajor=$(grep -Po -- 'version \K\w*' <<< $gzversion)
gzversionminor=$(grep -Po -- 'version \w*\S\K\w*' <<< $gzversion)
if [ $ROS_DISTRO == melodic ]; then
  if [ $gzversionmajor -eq 9 ] && [ $gzversionminor -lt 19 ]; then
  echo "You need the newest version of Gazebo. Installing now."
  sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
  wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
  sudo apt update
  sudo apt upgrade #--only-upgrade install gazebo
  else
  echo "Gazebo version is up to date ($gzversionmajor.$gzversionminor)."
  fi
else
  if [ $gzversionmajor -eq 11 ] && [ $gzversionminor -lt 9 ]; then
  echo "You need the newest version of Gazebo. PLEASE INSTALL AFTER THIS SCRIPT EXITS."
  else
  echo "Gazebo version is up to date ($gzversionmajor.$gzversionminor)."
  fi
fi

echo ""
echo -e "${cyan}${underline}Compiling additional packages...${clearformat}"
echo "NO ADDITIONAL PACKAGES TO COMPILE"




if [ ! -f /etc/udev/rules.d/10-sdrcJoystickRules ]; then
    echo ""
    echo "SDRC Joystick udev Rules Not Fould."
    echo -e "${format}Do you want to install joystick support files?${clearformat}"
    if [ $test_mode == "False" ]; then
      read -p 'y/n: ' cont
      case $cont in  
        y|Y) cont=True ;; 
        n|N) cont=False ;; 
        *) cont=False ;; 
      esac
    else
      cont=True
      echo "y"
    fi

    if [ $cont == "True" ]; then
      echo -e "${cyan}${underline}Intalling joystick udev rules...${clearformat}"
      sudo cp ./extras/etc/udev/rules.d/10-sdrcJoystick.rules /etc/udev/rules.d/
      echo "Installed joystick udev rules to /etc/udev/rules.d/10-sdrcJoystickRules."
    else
      echo "Not installing joystick udev rules."
    fi
fi




echo -e "${cyan}${underline}Running catkin_make:${clearformat}"
echo "cd $CATKIN"
cd $CATKIN
echo "catkin_make"
catkin_make

echo ""
echo "Successfully installed all dependencies!"


if [ ! -f ./sdrc_urc_restricted/README.md ]; then
    echo ""
    echo "SDRC URC Secret folder not initialized."
    echo -e "${format}Do you want to initialize and update this folder? (Only members of the SDRC GitHub Organization will be able to access these files)${clearformat}"
    if [ $test_mode == "False" ]; then
      read -p 'y/n: ' cont
      case $cont in  
        y|Y) cont=True ;; 
        n|N) cont=False ;; 
        *) cont=False ;; 
      esac
    else
      cont=True
      echo "y"
    fi

    if [ $cont == "True" ]; then
      echo -e "${cyan}${underline}Initializing and updating SDRC URC Secret folder...${clearformat}"
      git submodule update --init ./sdrc_urc_secret
      echo "Initialized and updated the files in ./sdrc_urc_secret/"
    else
      echo "Not initializing the SDRC URC Secret files."
    fi
fi

read -n 1 -s -r -p "Press any key to continue"
# clear
echo -e "\e[?1049l"

# echo This script will install dependencies needed to successfully run
# echo SDRC software with ROS.
# echo
# echo -e "\e[93m\e[1m\e[4mDo you wish to install the following dependencies:\e[0m"

# if [ ! -d "$CATKIN/src/roboteq" ]; then
# printf "\troboteq_driver\n"
# let "ntoinstall++"
# fi
# if [ ! -d "$CATKIN/src/serial" ]; then
# printf "\tserial\n"
# let "ntoinstall++"
# fi
# #printf "\n\tar_track_alvar"
# if [ ! -d "$CATKIN/src/joystick_drivers" ]; then
# printf "\tjoy\n"
# let "ntoinstall++"
# fi
# if ! dpkg -l | grep -qw libspnav-dev; then
#   printf "\tlibspnav-dev\n"
#   let "ntoinstall++"
# fi
# if ! dpkg -l | grep -qw libbluetooth-dev; then
#   printf "\tlibbluetooth-dev\n"
#   let "ntoinstall++"
# fi
# if ! dpkg -l | grep -qw libcwiid-dev; then
#   printf "\tlibcwiid-dev\n"
#   let "ntoinstall++"
# fi
# if ! dpkg -l | grep -qw libusb-dev; then
#   printf "\tlibusb-dev\n"
#   let "ntoinstall++"
# fi
# if ! dpkg -l | grep -qw ros-melodic-diagnostics; then
#   printf "\tros-melodic-diagnostics\n"
#   let "ntoinstall++"
# fi
# if ! dpkg -l | grep -qw ros-melodic-roslint; then
#   printf "\tros-melodic-roslint\n"
#   let "ntoinstall++"
# fi
# if ! dpkg -l | grep -qw ros-melodic-turtlesim; then
#   printf "\tros-melodic-turtlesim\n"
#   let "ntoinstall++"
# fi
# if ! dpkg -l | grep -qw libx11-dev; then
#   printf "\tlibx11-dev\n"
#   let "ntoinstall++"
# fi

# if [ $ntoinstall == 0 ]; then
#   echo -e "\e[1m\nYou are all up to date! Exiting..."
#   exit 0
# fi

# if [ $# -eq 0 ]; then
#   read -p 'y/n: ' continue
# fi

# if [ $1 = "-f" ] || ping -q -c 1 -W 1 8.8.8.8 >/dev/null; then
#   if [ $1 = "-f" ] || [ $continue = "y" ] || [ $continue = "Y" ] || [ $continue = "yes" ] || [ $continue = "Yes" ]
#   then

#     echo -e "\e[93m\e[4mUpdating repository:\e[0m"
#     echo apt-get update
#     sudo apt-get update
#     echo

#     if [ ! -d "$CATKIN/src" ]; then
#       echo "\n\e[93m\e[1mERROR: Cannot locate catkin workspace!"
#       echo "Make sure it is set up at $CATKIN"
#       exit 1
#     fi

#     # if [ ! -d "$CATKIN/src/roboteq" ]; then
#     #   echo -e "\e[93m\e[4mInstalling roboteq_driver:\e[0m"
#     #   cd $CATKIN/src/
#     #   git clone https://github.com/g/roboteq.git
#     #   echo
#     # fi

#     if [ ! -d "$CATKIN/src/serial" ]; then
#       echo -e "\e[93m\e[4mInstalling serial:\e[0m"
#       cd $CATKIN/src/
#       git clone https://github.com/wjwwood/serial.git
#       cd serial
#       make
#       echo
#     fi

#     if [ ! -d "$CATKIN/src/joystick_drivers" ]; then
#       echo -e "\e[93m\e[4mInstalling joy:\e[0m"
#       cd $CATKIN/src/
#       git clone https://github.com/ros-drivers/joystick_drivers.git
#       echo
#     fi

#     echo -e "\e[93m\e[4mInstalling libspnav-dev:\e[0m"
#     echo apt install -y libspnav-dev
#     sudo apt install -y libspnav-dev
#     echo
#     echo -e "\e[93m\e[4mInstalling libbluetooth-dev:\e[0m"
#     echo apt install -y libbluetooth-dev
#     sudo apt install -y libbluetooth-dev
#     echo
#     echo -e "\e[93m\e[4mInstalling libcwiid-dev:\e[0m"
#     echo apt install -y libcwiid-dev
#     sudo apt install -y libcwiid-dev
#     echo
#     echo -e "\e[93m\e[4mInstalling libusb-dev:\e[0m"
#     echo apt install -y libusb-dev
#     sudo apt install -y libusb-dev
#     echo
#     echo -e "\e[93m\e[4mInstalling ros-melodic-diagnostics:\e[0m"
#     echo apt install -y ros-melodic-diagnostics
#     sudo apt install -y ros-melodic-diagnostics
#     echo
#     echo -e "\e[93m\e[4mInstalling ros-melodic-roslint:\e[0m"
#     echo apt install -y ros-melodic-roslint
#     sudo apt install -y ros-melodic-roslint
#     echo
#     echo -e "\e[93m\e[4mInstalling ros-melodic-turtlesim:\e[0m"
#     echo apt install -y ros-melodic-turtlesim
#     sudo apt install -y ros-melodic-turtlesim
#     echo
#     echo -e "\e[93m\e[4mInstalling libx11-dev:\e[0m"
#     echo apt install -y libx11-dev
#     sudo apt install -y libx11-dev
#     echo

#     echo -e "\e[93m\e[4mRunning catkin_make:\e[0m"
#     echo cd $CATKIN
#     cd $CATKIN
#     echo catkin_make
#     catkin_make

#     echo
#     echo Successfully installed all dependencies!
#   fi
# else
#   echo -e "\n\e[93m\e[1mERROR! Please check your internet connection and try again!"
# fi