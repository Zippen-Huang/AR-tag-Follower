# AR-tag-Follower
A mecanum mobile platform  integrated ZED stereo camera follow ar_tag

HardWare： TX2，ZED camera，Arduino,MakeBlock ORION,
SoftWare:  ROS,Ar_track_alvar,rossrial

Specification:
STEP1: install ROS(http://wiki.ros.org/kinetic/Installation/Ubuntu)

STEP2: compile ar_track_alvar software package(http://wiki.ros.org/ar_track_alvar) or compile ar_track_alvar in ZED ros_wrapper(https://github.com/stereolabs/zed-ros-wrapper/tree/master/examples/zed_ar_track_alvar_example)

STEP3: new a rosoackage, and put ar_follower_mecanum.py code into new package, add executable authority

STEP4: install rosserial package(http://wiki.ros.org/rosserial)

STEP5: program download
       a. flash Mecanum_chassis code into ORION
       b. flash msgSendBySerial3 code into Arduino Mega2560
       c. Attention: load makeblockdrive zip into ORION, load ros_lib into Arduino

STEP6:TX2 connet to Arduino through usb-ttl, Arduino connet to ORION through Softserial lib

STEP7: start roscore and other program

here is my robot:
![Image text](https://github.com/Zippen-Huang/AR-tag-Follower/blob/master/QQ%E5%9B%BE%E7%89%8720181127221352.jpg)
      

