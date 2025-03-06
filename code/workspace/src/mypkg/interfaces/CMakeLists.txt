####设置项目基础属性####
cmake_minimum_required(VERSION 3.8) 	
project(mypkg_interfaces)					
########设置项目基础属性########

########找到外部依赖包########
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
########找到外部依赖包########

########信息########
ament_auto_generate_code()
########信息########

########MSG########
rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/Data.msg"
  "msg/Result.msg"
  "msg/MoveData.msg"
  DEPENDENCIES
    std_msgs
    geometry_msgs
)
########MSG########
ament_package()
