cmake_minimum_required(VERSION 3.0.2)
project(hmi_qt)

find_package(catkin REQUIRED COMPONENTS)

catkin_python_setup()

catkin_package()

install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/hmi_qt
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
