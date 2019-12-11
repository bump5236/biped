# biped
The scripts for Running and Walking

## Usage
Create directory for git clone and download there

```
$mkdir ~/git
$cd git
$git clone https://github.com/bump5236/biped
```

## Build
To compile robot_assist,"dynamixel_sdk" is required.

```
$mv biped/robot_assist ~/catkin_ws/src
$mv biped/robot_arm ~/catkin_ws/src
$cd catkin_ws
$catkin_make
```

## Notice
If you get error, it might be an error about "dynamixel_sdk".
One of the causes is absence of "dynamixel_sdk".
Please refer to the following URL.
http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/library_setup/cpp_linux/#cpp-linux

