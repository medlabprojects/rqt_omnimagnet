cmake_minimum_required(VERSION 3.1)
project(rqt_omnimagnet)

## Compile as C++14
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rqt_gui
  rqt_gui_cpp
  medlab_motor_control_board
)

find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)

find_package(Eigen3 REQUIRED NO_MODULE)
set(${PROJECT_NAME}_INCLUDE_DIRECTORIES
    include
    ${EIGEN3_INCLUDE_DIR}
)

set(${PROJECT_NAME}_SRCS
    src/rqt_omnimagnet.cpp
    src/omnimag_ros.cpp
    src/OmniMagnet.cpp
)

# location of header files
set(${PROJECT_NAME}_INCLUDES
  include
  ${EIGEN3_INCLUDE_DIR}
)

# NOTE: only need to list headers that require moc'ing (i.e. those that inherit QObject)
set(${PROJECT_NAME}_MOCS
    include/rqt_omnimagnet/rqt_omnimagnet.h
    include/rqt_omnimagnet/omnimag_ros.h
)

set(${PROJECT_NAME}_UIS
    src/omnimag_gui.ui
)

catkin_package(
  INCLUDE_DIRS ${${PROJECT_NAME}_INCLUDES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rqt_gui rqt_gui_cpp medlab_motor_control_board
)

qt5_wrap_cpp(${PROJECT_NAME}_MOCS_H ${${PROJECT_NAME}_MOCS})
qt5_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})


include_directories(
    ${${PROJECT_NAME}_INCLUDES}
    "${CMAKE_CURRENT_BINARY_DIR}"
	${catkin_INCLUDE_DIRS}
	${Qt5Widgets_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    ${${PROJECT_NAME}_SRCS}
    ${${PROJECT_NAME}_MOCS_H}
    ${${PROJECT_NAME}_UIS_H}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Widgets
)

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY 
	include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
