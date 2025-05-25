cmake_minimum_required(VERSION 3.8)
project(lab1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencias de build
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)


# --- Registra tu paquete Python
ament_python_install_package(${PROJECT_NAME})



file(GLOB PY_SCRIPTS
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py"
)

install(
  PROGRAMS
    ${PY_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

# --- Instala los scripts como ejecutables
install(
  PROGRAMS
    scripts/dead_reckoning_nav.py
    scripts/obstacle_detector.py
    scripts/pose_loader.py
    scripts/data_recorder.py
    scripts/teleop_nav.py
    scripts/obstacle_detector.py
  DESTINATION lib/${PROJECT_NAME}
)


# --- Instala los launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Instalo la carpeta config el archivo .txt
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
