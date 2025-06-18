cmake_minimum_required(VERSION 3.5)
project(v2x_stack_btp)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ros_etsi_its_msgs REQUIRED)
find_package(etsi_its_msgs REQUIRED)
find_package(etsi_its_vam_ts_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(udp_msgs REQUIRED)

set(VANETZA_INSTALL ON)
set(VANETZA_EXPORT_PACKAGE OFF)
set(BUILD_SHARED_LIBS ON)
add_subdirectory(extern/vanetza)
mark_as_advanced(BUILD_BENCHMARK BUILD_CERTIFY BUILD_SOCKTAP BUILD_TESTS BUILD_USING_CONAN)
mark_as_advanced(VANETZA_ASN1_WITH_ASN1C VANETZA_EXPORT_PACKAGE VANETZA_NET_WITH_POSIX)

# Define custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Area.msg"
  "msg/Location.msg"
  "msg/CohdaInd.msg"
  "msg/CohdaReq.msg"
  "msg/UDPDataIndHdr.msg"
  "msg/UDPMsgType.msg"
  "msg/TrafficClass.msg"
  "msg/GeoNetDestination.msg"
  "msg/GeoNetArea.msg"
  "msg/BtpDataIndication.msg"
  "srv/BtpData.srv"
  DEPENDENCIES std_msgs
)

# Include directories
include_directories(
	include
)

# Add executable UDP Dispacher
add_executable(${PROJECT_NAME}_udp_dispacher_node    
    src/udp_dispatcher.cpp
)

#Link against dependencies UDP Dispacher
ament_target_dependencies(${PROJECT_NAME}_udp_dispacher_node  rclcpp rclcpp_components std_msgs udp_msgs)

# Link against generated messages UDP Dispacher
rosidl_target_interfaces(${PROJECT_NAME}_udp_dispacher_node  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install targets
install(TARGETS
${PROJECT_NAME}_udp_dispacher_node
  DESTINATION lib/${PROJECT_NAME}
)

# Add executable
add_executable(${PROJECT_NAME}_btp_data_node
    src/btp_data_service.cpp    
)

# Include directories for the executable
target_include_directories(${PROJECT_NAME}_btp_data_node
    PUBLIC
    include
    ${v2x_stack_INCLUDE_DIRS}
    ${ros_etsi_its_msgs_INCLUDE_DIRS}
)

# Link against Vanetza libraries
target_link_libraries(${PROJECT_NAME}_btp_data_node Vanetza::vanetza)

# Link against dependencies
ament_target_dependencies(${PROJECT_NAME}_btp_data_node
    rclcpp
    rclcpp_components
    std_msgs    
    ros_etsi_its_msgs
)

# Link against generated messages
rosidl_target_interfaces(${PROJECT_NAME}_btp_data_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install targets
install(TARGETS ${PROJECT_NAME}_btp_data_node
    DESTINATION lib/${PROJECT_NAME}
)

##CAM
# Add executable
add_executable(${PROJECT_NAME}_ca_node
    src/ca_message.cpp
    src/ca_rx.cpp
)

# Include directories for the executable
target_include_directories(${PROJECT_NAME}_ca_node
    PUBLIC
    include
    ${v2x_stack_INCLUDE_DIRS}
    ${ros_etsi_its_msgs_INCLUDE_DIRS}
)

# Link against dependencies
ament_target_dependencies(${PROJECT_NAME}_ca_node
    rclcpp
    rclcpp_components
    std_msgs
    #v2x_stack
    ros_etsi_its_msgs
)

# Link against generated messages
rosidl_target_interfaces(${PROJECT_NAME}_ca_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Link against Vanetza libraries
target_link_libraries(${PROJECT_NAME}_ca_node Vanetza::asn1 Vanetza::facilities)


# Install targets
install(TARGETS ${PROJECT_NAME}_ca_node
    DESTINATION lib/${PROJECT_NAME}
)


##VAM
# Add executable
add_executable(${PROJECT_NAME}_va_node src/vam_tx.cpp)

# Include directories
target_include_directories(${PROJECT_NAME}_va_node 
    PUBLIC 
    include 
    ${v2x_stack_INCLUDE_DIRS} # Is this the right directory or do we need to put the headerfiles into "include/" folder?
    ${ros_etsi_its_msgs_INCLUDE_DIRS}
)

# Link against dependencies
ament_target_dependencies(${PROJECT_NAME}_va_node
    rclcpp
    rclcpp_components
    std_msgs
    #v2x_stack
    ros_etsi_its_msgs
    sensor_msgs
    etsi_its_msgs
    etsi_its_vam_ts_msgs
)

# Link against generated messages
rosidl_target_interfaces(${PROJECT_NAME}_va_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Link against Vanetza libraries
target_link_libraries(${PROJECT_NAME}_va_node Vanetza::asn1 Vanetza::facilities)

# Install targets
install(TARGETS ${PROJECT_NAME}_va_node
    DESTINATION lib/${PROJECT_NAME}
)

##CPM 
# Add executable
add_executable(${PROJECT_NAME}_cp_node
    src/cp_message.cpp
    src/cp_rx.cpp
)

# Include directories for the executable
target_include_directories(${PROJECT_NAME}_cp_node
    PUBLIC
    include
    ${v2x_stack_INCLUDE_DIRS}
    ${ros_etsi_its_msgs_INCLUDE_DIRS}
)

# Link against dependencies
ament_target_dependencies(${PROJECT_NAME}_cp_node
    rclcpp
    rclcpp_components
    std_msgs
    #v2x_stack
    ros_etsi_its_msgs
)

# Link against generated messages
rosidl_target_interfaces(${PROJECT_NAME}_cp_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Link against Vanetza libraries
target_link_libraries(${PROJECT_NAME}_cp_node Vanetza::asn1 Vanetza::facilities)

# Install targets
install(TARGETS ${PROJECT_NAME}_cp_node
    DESTINATION lib/${PROJECT_NAME}
)

##DENM
# Add executable
add_executable(${PROJECT_NAME}_den_node
    src/den_message.cpp
    src/den_rx.cpp
)

# Include directories for the executable
target_include_directories(${PROJECT_NAME}_den_node
    PUBLIC
    include
    ${v2x_stack_INCLUDE_DIRS}
    ${ros_etsi_its_msgs_INCLUDE_DIRS}
)

# Link against dependencies
ament_target_dependencies(${PROJECT_NAME}_den_node
    rclcpp
    rclcpp_components
    std_msgs
    #v2x_stack
    ros_etsi_its_msgs
)

# Link against generated messages
rosidl_target_interfaces(${PROJECT_NAME}_den_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Link against Vanetza libraries
target_link_libraries(${PROJECT_NAME}_den_node Vanetza::asn1 Vanetza::facilities)


# Install targets
install(TARGETS ${PROJECT_NAME}_den_node
    DESTINATION lib/${PROJECT_NAME}
)

##CPM_TS
# Add executable
add_executable(${PROJECT_NAME}_cp_ts_node
    src/cp_ts_message.cpp
    src/cp_ts_rx.cpp
)

# Include directories for the executable
target_include_directories(${PROJECT_NAME}_cp_ts_node
    PUBLIC
    include
    ${v2x_stack_INCLUDE_DIRS}
    ${ros_etsi_its_msgs_INCLUDE_DIRS}
    extern/vanetza/asn1/its/r2
)

# Link against dependencies
ament_target_dependencies(${PROJECT_NAME}_cp_ts_node
    rclcpp
    rclcpp_components
    std_msgs
    #v2x_stack
    etsi_its_msgs
)

# Link against generated messages
rosidl_target_interfaces(${PROJECT_NAME}_cp_ts_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Link against Vanetza libraries
target_link_libraries(${PROJECT_NAME}_cp_ts_node Vanetza::vanetza)

# Install targets
install(TARGETS ${PROJECT_NAME}_cp_ts_node
    DESTINATION lib/${PROJECT_NAME}
)



# Install include files
install(DIRECTORY include/
    DESTINATION include/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)


# Register the package as an ament package
ament_package()
include_directories(/home/vboxuser/colcon_ws/src/v2x_stack/extern/vanetza)
