# crazy is a ros package and can be used to control 2 or more crazyflie vehicles, which is based on vicon.Walt, lucasyu17, and me are all contributors to this repository.

----------------------------------------------------------------------------------------
# notes for myself:
## 1 catkin_make error:undefined reference to `cv::*'
  solution: add these into cmakelist:
	FIND_PACKAGE( OpenCV REQUIRED )
	add_executable( main main.cpp )
	TARGET_LINK_LIBRARIES( main ${OpenCV_LIBS} ) 
## 2 add a flight state 
