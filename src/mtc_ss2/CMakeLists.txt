cmake_minimum_required(VERSION 3.8)
project(mtc_ss2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# mtc_ss2.cpp
add_executable(mtc_ss2 src/mtc_ss2.cpp)
ament_target_dependencies(mtc_ss2 moveit_task_constructor_core rclcpp tf2 tf2_geometry_msgs geometry_msgs std_msgs)
target_include_directories(mtc_ss2 PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_compile_features(mtc_ss2 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# retrieval_ss2
add_executable(retrieval_ss2 src/retrieval_ss2.cpp)
ament_target_dependencies(retrieval_ss2
  moveit_task_constructor_core
  rclcpp
  tf2
  tf2_geometry_msgs
  geometry_msgs
  std_msgs
)
target_include_directories(retrieval_ss2 PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_compile_features(retrieval_ss2 PUBLIC c_std_99 cxx_std_17)

install(TARGETS mtc_ss2 retrieval_ss2
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
# the following line skips the linter which checks for copyrights
# uncomment the line when a copyright and license is not present in all source files
#set(ament_cmake_copyright_FOUND TRUE)
# the following line skips cpplint (only works in a git repo)
# uncomment the line when this package is not in a git repo
#set(ament_cmake_cpplint_FOUND TRUE)
ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()