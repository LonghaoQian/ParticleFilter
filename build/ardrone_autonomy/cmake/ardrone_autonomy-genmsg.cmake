# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ardrone_autonomy: 32 messages, 4 services")

set(MSG_I_FLAGS "-Iardrone_autonomy:/home/longhao/catkin_ws/src/ardrone_autonomy/msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ardrone_autonomy_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_references.msg" NAME_WE)
add_custom_target(_ardrone_autonomy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_autonomy" "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_references.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_wind_speed.msg" NAME_WE)
add_custom_target(_ardrone_autonomy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_autonomy" "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_wind_speed.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_rc_references.msg" NAME_WE)
add_custom_target(_ardrone_autonomy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_autonomy" "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_rc_references.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_magneto.msg" NAME_WE)
add_custom_target(_ardrone_autonomy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_autonomy" "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_magneto.msg" "ardrone_autonomy/vector31:std_msgs/Header"
)

get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_games.msg" NAME_WE)
add_custom_target(_ardrone_autonomy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_autonomy" "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_games.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_demo.msg" NAME_WE)
add_custom_target(_ardrone_autonomy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_autonomy" "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_demo.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_wifi.msg" NAME_WE)
add_custom_target(_ardrone_autonomy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_autonomy" "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_wifi.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_hdvideo_stream.msg" NAME_WE)
add_custom_target(_ardrone_autonomy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_autonomy" "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_hdvideo_stream.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/Navdata.msg" NAME_WE)
add_custom_target(_ardrone_autonomy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_autonomy" "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/Navdata.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_euler_angles.msg" NAME_WE)
add_custom_target(_ardrone_autonomy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_autonomy" "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_euler_angles.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_kalman_pressure.msg" NAME_WE)
add_custom_target(_ardrone_autonomy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_autonomy" "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_kalman_pressure.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/FlightAnim.srv" NAME_WE)
add_custom_target(_ardrone_autonomy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_autonomy" "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/FlightAnim.srv" ""
)

get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/CamSelect.srv" NAME_WE)
add_custom_target(_ardrone_autonomy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_autonomy" "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/CamSelect.srv" ""
)

get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_raw.msg" NAME_WE)
add_custom_target(_ardrone_autonomy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_autonomy" "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_raw.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_pwm.msg" NAME_WE)
add_custom_target(_ardrone_autonomy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_autonomy" "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_pwm.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_trims.msg" NAME_WE)
add_custom_target(_ardrone_autonomy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_autonomy" "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_trims.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_watchdog.msg" NAME_WE)
add_custom_target(_ardrone_autonomy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_autonomy" "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_watchdog.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_pressure_raw.msg" NAME_WE)
add_custom_target(_ardrone_autonomy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_autonomy" "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_pressure_raw.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_raw_measures.msg" NAME_WE)
add_custom_target(_ardrone_autonomy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_autonomy" "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_raw_measures.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_perf.msg" NAME_WE)
add_custom_target(_ardrone_autonomy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_autonomy" "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_perf.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/LedAnim.srv" NAME_WE)
add_custom_target(_ardrone_autonomy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_autonomy" "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/LedAnim.srv" ""
)

get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_zimmu_3000.msg" NAME_WE)
add_custom_target(_ardrone_autonomy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_autonomy" "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_zimmu_3000.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_trackers_send.msg" NAME_WE)
add_custom_target(_ardrone_autonomy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_autonomy" "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_trackers_send.msg" "ardrone_autonomy/vector21:std_msgs/Header"
)

get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_video_stream.msg" NAME_WE)
add_custom_target(_ardrone_autonomy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_autonomy" "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_video_stream.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_of.msg" NAME_WE)
add_custom_target(_ardrone_autonomy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_autonomy" "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_of.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision.msg" NAME_WE)
add_custom_target(_ardrone_autonomy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_autonomy" "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision.msg" "ardrone_autonomy/vector31:std_msgs/Header"
)

get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_adc_data_frame.msg" NAME_WE)
add_custom_target(_ardrone_autonomy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_autonomy" "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_adc_data_frame.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_phys_measures.msg" NAME_WE)
add_custom_target(_ardrone_autonomy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_autonomy" "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_phys_measures.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector21.msg" NAME_WE)
add_custom_target(_ardrone_autonomy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_autonomy" "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector21.msg" ""
)

get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/matrix33.msg" NAME_WE)
add_custom_target(_ardrone_autonomy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_autonomy" "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/matrix33.msg" ""
)

get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_detect.msg" NAME_WE)
add_custom_target(_ardrone_autonomy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_autonomy" "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_detect.msg" "ardrone_autonomy/vector31:std_msgs/Header:ardrone_autonomy/matrix33"
)

get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_altitude.msg" NAME_WE)
add_custom_target(_ardrone_autonomy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_autonomy" "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_altitude.msg" "ardrone_autonomy/vector21:ardrone_autonomy/vector31:std_msgs/Header"
)

get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_time.msg" NAME_WE)
add_custom_target(_ardrone_autonomy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_autonomy" "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_time.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/RecordEnable.srv" NAME_WE)
add_custom_target(_ardrone_autonomy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_autonomy" "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/RecordEnable.srv" ""
)

get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector31.msg" NAME_WE)
add_custom_target(_ardrone_autonomy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_autonomy" "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector31.msg" ""
)

get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_gyros_offsets.msg" NAME_WE)
add_custom_target(_ardrone_autonomy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_autonomy" "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_gyros_offsets.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_references.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_cpp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_of.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_cpp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_wind_speed.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_cpp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_rc_references.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_cpp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_magneto.msg"
  "${MSG_I_FLAGS}"
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector31.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_cpp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_games.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_cpp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_demo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_cpp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_wifi.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_cpp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_hdvideo_stream.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_cpp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/Navdata.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_cpp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_euler_angles.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_cpp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/matrix33.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_cpp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_raw.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_cpp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_pwm.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_cpp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_video_stream.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_cpp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_watchdog.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_cpp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_pressure_raw.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_cpp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_raw_measures.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_cpp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_perf.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_cpp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_zimmu_3000.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_cpp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_trackers_send.msg"
  "${MSG_I_FLAGS}"
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector21.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_cpp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_trims.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_cpp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_phys_measures.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_cpp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision.msg"
  "${MSG_I_FLAGS}"
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector31.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_cpp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_adc_data_frame.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_cpp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector21.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_cpp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_kalman_pressure.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_cpp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_detect.msg"
  "${MSG_I_FLAGS}"
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector31.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/longhao/catkin_ws/src/ardrone_autonomy/msg/matrix33.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_cpp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_altitude.msg"
  "${MSG_I_FLAGS}"
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector21.msg;/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector31.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_cpp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_time.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_cpp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector31.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_cpp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_gyros_offsets.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
)

### Generating Services
_generate_srv_cpp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/CamSelect.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
)
_generate_srv_cpp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/RecordEnable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
)
_generate_srv_cpp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/FlightAnim.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
)
_generate_srv_cpp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/LedAnim.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
)

### Generating Module File
_generate_module_cpp(ardrone_autonomy
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ardrone_autonomy_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ardrone_autonomy_generate_messages ardrone_autonomy_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_references.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_cpp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_wind_speed.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_cpp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_rc_references.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_cpp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_magneto.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_cpp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_games.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_cpp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_demo.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_cpp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_wifi.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_cpp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_hdvideo_stream.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_cpp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/Navdata.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_cpp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_euler_angles.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_cpp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_kalman_pressure.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_cpp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/FlightAnim.srv" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_cpp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/CamSelect.srv" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_cpp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_raw.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_cpp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_pwm.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_cpp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_trims.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_cpp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_watchdog.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_cpp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_pressure_raw.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_cpp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_raw_measures.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_cpp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_perf.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_cpp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/LedAnim.srv" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_cpp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_zimmu_3000.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_cpp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_trackers_send.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_cpp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_video_stream.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_cpp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_of.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_cpp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_cpp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_adc_data_frame.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_cpp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_phys_measures.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_cpp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector21.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_cpp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/matrix33.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_cpp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_detect.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_cpp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_altitude.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_cpp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_time.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_cpp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/RecordEnable.srv" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_cpp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector31.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_cpp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_gyros_offsets.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_cpp _ardrone_autonomy_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ardrone_autonomy_gencpp)
add_dependencies(ardrone_autonomy_gencpp ardrone_autonomy_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ardrone_autonomy_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_references.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_eus(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_of.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_eus(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_wind_speed.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_eus(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_rc_references.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_eus(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_magneto.msg"
  "${MSG_I_FLAGS}"
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector31.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_eus(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_games.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_eus(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_demo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_eus(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_wifi.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_eus(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_hdvideo_stream.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_eus(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/Navdata.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_eus(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_euler_angles.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_eus(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/matrix33.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_eus(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_raw.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_eus(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_pwm.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_eus(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_video_stream.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_eus(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_watchdog.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_eus(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_pressure_raw.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_eus(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_raw_measures.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_eus(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_perf.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_eus(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_zimmu_3000.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_eus(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_trackers_send.msg"
  "${MSG_I_FLAGS}"
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector21.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_eus(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_trims.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_eus(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_phys_measures.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_eus(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision.msg"
  "${MSG_I_FLAGS}"
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector31.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_eus(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_adc_data_frame.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_eus(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector21.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_eus(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_kalman_pressure.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_eus(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_detect.msg"
  "${MSG_I_FLAGS}"
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector31.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/longhao/catkin_ws/src/ardrone_autonomy/msg/matrix33.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_eus(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_altitude.msg"
  "${MSG_I_FLAGS}"
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector21.msg;/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector31.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_eus(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_time.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_eus(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector31.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_eus(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_gyros_offsets.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
)

### Generating Services
_generate_srv_eus(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/CamSelect.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
)
_generate_srv_eus(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/RecordEnable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
)
_generate_srv_eus(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/FlightAnim.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
)
_generate_srv_eus(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/LedAnim.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
)

### Generating Module File
_generate_module_eus(ardrone_autonomy
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(ardrone_autonomy_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(ardrone_autonomy_generate_messages ardrone_autonomy_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_references.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_eus _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_wind_speed.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_eus _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_rc_references.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_eus _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_magneto.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_eus _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_games.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_eus _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_demo.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_eus _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_wifi.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_eus _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_hdvideo_stream.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_eus _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/Navdata.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_eus _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_euler_angles.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_eus _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_kalman_pressure.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_eus _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/FlightAnim.srv" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_eus _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/CamSelect.srv" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_eus _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_raw.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_eus _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_pwm.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_eus _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_trims.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_eus _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_watchdog.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_eus _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_pressure_raw.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_eus _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_raw_measures.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_eus _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_perf.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_eus _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/LedAnim.srv" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_eus _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_zimmu_3000.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_eus _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_trackers_send.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_eus _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_video_stream.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_eus _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_of.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_eus _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_eus _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_adc_data_frame.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_eus _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_phys_measures.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_eus _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector21.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_eus _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/matrix33.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_eus _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_detect.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_eus _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_altitude.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_eus _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_time.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_eus _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/RecordEnable.srv" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_eus _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector31.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_eus _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_gyros_offsets.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_eus _ardrone_autonomy_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ardrone_autonomy_geneus)
add_dependencies(ardrone_autonomy_geneus ardrone_autonomy_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ardrone_autonomy_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_references.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_lisp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_of.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_lisp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_wind_speed.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_lisp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_rc_references.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_lisp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_magneto.msg"
  "${MSG_I_FLAGS}"
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector31.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_lisp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_games.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_lisp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_demo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_lisp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_wifi.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_lisp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_hdvideo_stream.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_lisp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/Navdata.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_lisp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_euler_angles.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_lisp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/matrix33.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_lisp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_raw.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_lisp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_pwm.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_lisp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_video_stream.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_lisp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_watchdog.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_lisp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_pressure_raw.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_lisp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_raw_measures.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_lisp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_perf.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_lisp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_zimmu_3000.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_lisp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_trackers_send.msg"
  "${MSG_I_FLAGS}"
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector21.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_lisp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_trims.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_lisp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_phys_measures.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_lisp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision.msg"
  "${MSG_I_FLAGS}"
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector31.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_lisp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_adc_data_frame.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_lisp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector21.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_lisp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_kalman_pressure.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_lisp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_detect.msg"
  "${MSG_I_FLAGS}"
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector31.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/longhao/catkin_ws/src/ardrone_autonomy/msg/matrix33.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_lisp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_altitude.msg"
  "${MSG_I_FLAGS}"
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector21.msg;/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector31.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_lisp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_time.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_lisp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector31.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_lisp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_gyros_offsets.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
)

### Generating Services
_generate_srv_lisp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/CamSelect.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
)
_generate_srv_lisp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/RecordEnable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
)
_generate_srv_lisp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/FlightAnim.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
)
_generate_srv_lisp(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/LedAnim.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
)

### Generating Module File
_generate_module_lisp(ardrone_autonomy
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ardrone_autonomy_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ardrone_autonomy_generate_messages ardrone_autonomy_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_references.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_lisp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_wind_speed.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_lisp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_rc_references.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_lisp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_magneto.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_lisp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_games.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_lisp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_demo.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_lisp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_wifi.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_lisp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_hdvideo_stream.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_lisp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/Navdata.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_lisp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_euler_angles.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_lisp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_kalman_pressure.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_lisp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/FlightAnim.srv" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_lisp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/CamSelect.srv" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_lisp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_raw.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_lisp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_pwm.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_lisp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_trims.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_lisp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_watchdog.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_lisp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_pressure_raw.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_lisp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_raw_measures.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_lisp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_perf.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_lisp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/LedAnim.srv" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_lisp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_zimmu_3000.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_lisp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_trackers_send.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_lisp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_video_stream.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_lisp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_of.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_lisp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_lisp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_adc_data_frame.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_lisp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_phys_measures.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_lisp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector21.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_lisp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/matrix33.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_lisp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_detect.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_lisp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_altitude.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_lisp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_time.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_lisp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/RecordEnable.srv" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_lisp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector31.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_lisp _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_gyros_offsets.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_lisp _ardrone_autonomy_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ardrone_autonomy_genlisp)
add_dependencies(ardrone_autonomy_genlisp ardrone_autonomy_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ardrone_autonomy_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_references.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_nodejs(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_of.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_nodejs(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_wind_speed.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_nodejs(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_rc_references.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_nodejs(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_magneto.msg"
  "${MSG_I_FLAGS}"
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector31.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_nodejs(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_games.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_nodejs(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_demo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_nodejs(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_wifi.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_nodejs(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_hdvideo_stream.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_nodejs(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/Navdata.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_nodejs(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_euler_angles.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_nodejs(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/matrix33.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_nodejs(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_raw.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_nodejs(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_pwm.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_nodejs(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_video_stream.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_nodejs(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_watchdog.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_nodejs(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_pressure_raw.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_nodejs(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_raw_measures.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_nodejs(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_perf.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_nodejs(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_zimmu_3000.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_nodejs(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_trackers_send.msg"
  "${MSG_I_FLAGS}"
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector21.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_nodejs(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_trims.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_nodejs(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_phys_measures.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_nodejs(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision.msg"
  "${MSG_I_FLAGS}"
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector31.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_nodejs(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_adc_data_frame.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_nodejs(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector21.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_nodejs(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_kalman_pressure.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_nodejs(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_detect.msg"
  "${MSG_I_FLAGS}"
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector31.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/longhao/catkin_ws/src/ardrone_autonomy/msg/matrix33.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_nodejs(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_altitude.msg"
  "${MSG_I_FLAGS}"
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector21.msg;/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector31.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_nodejs(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_time.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_nodejs(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector31.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_nodejs(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_gyros_offsets.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
)

### Generating Services
_generate_srv_nodejs(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/CamSelect.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
)
_generate_srv_nodejs(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/RecordEnable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
)
_generate_srv_nodejs(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/FlightAnim.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
)
_generate_srv_nodejs(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/LedAnim.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
)

### Generating Module File
_generate_module_nodejs(ardrone_autonomy
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(ardrone_autonomy_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(ardrone_autonomy_generate_messages ardrone_autonomy_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_references.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_nodejs _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_wind_speed.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_nodejs _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_rc_references.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_nodejs _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_magneto.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_nodejs _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_games.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_nodejs _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_demo.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_nodejs _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_wifi.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_nodejs _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_hdvideo_stream.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_nodejs _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/Navdata.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_nodejs _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_euler_angles.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_nodejs _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_kalman_pressure.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_nodejs _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/FlightAnim.srv" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_nodejs _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/CamSelect.srv" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_nodejs _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_raw.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_nodejs _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_pwm.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_nodejs _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_trims.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_nodejs _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_watchdog.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_nodejs _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_pressure_raw.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_nodejs _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_raw_measures.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_nodejs _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_perf.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_nodejs _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/LedAnim.srv" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_nodejs _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_zimmu_3000.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_nodejs _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_trackers_send.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_nodejs _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_video_stream.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_nodejs _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_of.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_nodejs _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_nodejs _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_adc_data_frame.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_nodejs _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_phys_measures.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_nodejs _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector21.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_nodejs _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/matrix33.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_nodejs _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_detect.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_nodejs _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_altitude.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_nodejs _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_time.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_nodejs _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/RecordEnable.srv" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_nodejs _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector31.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_nodejs _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_gyros_offsets.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_nodejs _ardrone_autonomy_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ardrone_autonomy_gennodejs)
add_dependencies(ardrone_autonomy_gennodejs ardrone_autonomy_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ardrone_autonomy_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_references.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_py(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_of.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_py(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_wind_speed.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_py(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_rc_references.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_py(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_magneto.msg"
  "${MSG_I_FLAGS}"
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector31.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_py(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_games.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_py(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_demo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_py(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_wifi.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_py(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_hdvideo_stream.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_py(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/Navdata.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_py(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_euler_angles.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_py(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/matrix33.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_py(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_raw.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_py(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_pwm.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_py(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_video_stream.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_py(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_watchdog.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_py(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_pressure_raw.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_py(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_raw_measures.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_py(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_perf.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_py(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_zimmu_3000.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_py(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_trackers_send.msg"
  "${MSG_I_FLAGS}"
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector21.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_py(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_trims.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_py(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_phys_measures.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_py(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision.msg"
  "${MSG_I_FLAGS}"
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector31.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_py(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_adc_data_frame.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_py(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector21.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_py(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_kalman_pressure.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_py(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_detect.msg"
  "${MSG_I_FLAGS}"
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector31.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/longhao/catkin_ws/src/ardrone_autonomy/msg/matrix33.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_py(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_altitude.msg"
  "${MSG_I_FLAGS}"
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector21.msg;/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector31.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_py(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_time.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_py(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector31.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
)
_generate_msg_py(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_gyros_offsets.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
)

### Generating Services
_generate_srv_py(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/CamSelect.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
)
_generate_srv_py(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/RecordEnable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
)
_generate_srv_py(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/FlightAnim.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
)
_generate_srv_py(ardrone_autonomy
  "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/LedAnim.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
)

### Generating Module File
_generate_module_py(ardrone_autonomy
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ardrone_autonomy_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ardrone_autonomy_generate_messages ardrone_autonomy_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_references.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_py _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_wind_speed.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_py _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_rc_references.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_py _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_magneto.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_py _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_games.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_py _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_demo.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_py _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_wifi.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_py _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_hdvideo_stream.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_py _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/Navdata.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_py _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_euler_angles.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_py _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_kalman_pressure.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_py _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/FlightAnim.srv" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_py _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/CamSelect.srv" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_py _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_raw.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_py _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_pwm.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_py _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_trims.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_py _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_watchdog.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_py _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_pressure_raw.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_py _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_raw_measures.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_py _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_perf.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_py _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/LedAnim.srv" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_py _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_zimmu_3000.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_py _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_trackers_send.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_py _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_video_stream.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_py _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_of.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_py _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_py _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_adc_data_frame.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_py _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_phys_measures.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_py _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector21.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_py _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/matrix33.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_py _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_vision_detect.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_py _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_altitude.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_py _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_time.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_py _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/srv/RecordEnable.srv" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_py _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/vector31.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_py _ardrone_autonomy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/longhao/catkin_ws/src/ardrone_autonomy/msg/navdata_gyros_offsets.msg" NAME_WE)
add_dependencies(ardrone_autonomy_generate_messages_py _ardrone_autonomy_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ardrone_autonomy_genpy)
add_dependencies(ardrone_autonomy_genpy ardrone_autonomy_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ardrone_autonomy_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_autonomy
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(ardrone_autonomy_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(ardrone_autonomy_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ardrone_autonomy
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(ardrone_autonomy_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(ardrone_autonomy_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_autonomy
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(ardrone_autonomy_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(ardrone_autonomy_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ardrone_autonomy
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(ardrone_autonomy_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(ardrone_autonomy_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_autonomy
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(ardrone_autonomy_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(ardrone_autonomy_generate_messages_py std_msgs_generate_messages_py)
endif()
