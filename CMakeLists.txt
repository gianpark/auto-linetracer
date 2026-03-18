cmake_minimum_required(VERSION 3.8)
project(linetracer_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(dxl3 REQUIRED)          # ← dxl3 패키지 추가

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

# ===== video_publisher 노드 =====
add_executable(video_publisher src/video_publisher.cpp)
ament_target_dependencies(video_publisher
  rclcpp sensor_msgs cv_bridge OpenCV
)
target_link_libraries(video_publisher ${OpenCV_LIBS})
target_compile_features(video_publisher PUBLIC cxx_std_17)

# ===== linetracer_sim 노드 =====
add_executable(linetracer_sim src/linetracer_sim.cpp)
ament_target_dependencies(linetracer_sim
  rclcpp sensor_msgs std_msgs cv_bridge image_transport OpenCV
  dxl3                                # ← dxl3 의존성 추가
)
target_link_libraries(linetracer_sim ${OpenCV_LIBS})
target_compile_features(linetracer_sim PUBLIC cxx_std_17)

# ===== 설치 =====
install(TARGETS
  video_publisher
  linetracer_sim
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
