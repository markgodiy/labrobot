cmake_minimum_required(VERSION 3.5)
project(labrobot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

install(
  DIRECTORY config description launch worlds scripts upython
  DESTINATION share/${PROJECT_NAME}
)

# Create empty lib directory to satisfy ROS 2 package structure requirements
install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory \${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME})")

# Ensure all .xacro files are installed
install(DIRECTORY description/
  DESTINATION share/${PROJECT_NAME}/description
  FILES_MATCHING PATTERN "*.xacro"
)

# Install MicroPython files
install(DIRECTORY upython/
  DESTINATION share/${PROJECT_NAME}/upython
  FILES_MATCHING PATTERN "*.py"
)

ament_package()
