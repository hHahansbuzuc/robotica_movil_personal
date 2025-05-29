cmake_minimum_required(VERSION 3.8)
project(lab2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Instala todos tus scripts Python como ejecutables
install(PROGRAMS
  scripts/desplazador.py
  scripts/pose_loader.py
  scripts/pub_velocidad.py
  scripts/pid.py
  scripts/run_all.py
  scripts/wall_detector.py
  scripts/data_recorder.py
  scripts/run_all_2.py
  scripts/run_all_3.py
  DESTINATION lib/${PROJECT_NAME}
)

file(GLOB PY_SCRIPTS
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py"
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


# Linter y pruebas (opcional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # estas dos líneas evitan errores si no tienes copyleft ni git
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
