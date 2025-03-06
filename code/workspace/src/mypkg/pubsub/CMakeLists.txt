# ####====================================####
# ####====set project basic properties====####
# ####====================================####
# 设置项目基础属性，cmake最小版本，项目名称，c/c++版本，编译选项等

cmake_minimum_required(VERSION 3.8)
project(mypkg_pubsub)

# c++17
set(CMAKE_CXX_STANDARD 17)
# no compiler extension
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set optimization level
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")

# ####====================####
# ####====set cpp file====####
# ####====================####
# 设置一些变量（宏），来指代文件

set(PUB src/pub.cpp)
set(SUB src/sub.cpp)
set(TARGET1 pub_exe)
set(TARGET2 sub_exe)

# ================================================####
# ====automatically find external dependencies====####
# ================================================####
# 自动查找依赖
find_package (ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies ()

# ==========================================####
# ====automatically add executable files====####
# ==========================================####
# 添加可执行文件，lib/exe

# 1.add lib
# ament_auto_add_library(${PROJECT_NAME} SHARED ${MAIN})
# 2.add exe
ament_auto_add_executable(${TARGET1}  ${PUB})
ament_auto_add_executable(${TARGET2}  ${SUB})

# ========================####
# ====additional links====####
# ========================####
# 添加ros找不到的依赖，如相机驱动等

# ex:dahen api
# target_link_libraries(${PROJECT_NAME} gxiapi)

# =====================####
# ====register node====####
# =====================####
# 可选使用容器注册多个节点

# rclcpp_components_register_node(${PROJECT_NAME} PLUGIN AcquisitionNode
                                # EXECUTABLE acquisition_node)

# ==================####
# ====build test====####
# ==================####
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights uncomment
  # the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE) the following line skips cpplint (only
  # works in a git repo) uncomment the line when this package is not in a git
  # repo set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# =========================####
# ====automatic install====####
# =========================####
# 配置文件，启动文件等（文件夹）
ament_auto_package(
	INSTALL_TO_SHARE
	config
	launch
)

#可选---添加可执行的py脚本
file(GLOB PYTHON_SCRIPTS scripts/*.py)
install(PROGRAMS
  ${PYTHON_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)
