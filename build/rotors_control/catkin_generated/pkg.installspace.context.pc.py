# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "geometry_msgs;mav_msgs;nav_msgs;roscpp;sensor_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lposition_controller;-lcrazyflie_onboard_controller;-lsensfusion6;-lcrazyflie_complementary_filter".split(';') if "-lposition_controller;-lcrazyflie_onboard_controller;-lsensfusion6;-lcrazyflie_complementary_filter" != "" else []
PROJECT_NAME = "rotors_control"
PROJECT_SPACE_DIR = "/home/chris/catkin_ws/install"
PROJECT_VERSION = "7.0.1"
