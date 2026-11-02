cmake_minimum_required(VERSION 3.22)
project(gz_plugins)

# Apply a local policy floor to legacy third-party CMake files (JsonCpp/GTest).
# The project's own minimum remains 3.22; no command-line workaround is needed.
if(CMAKE_VERSION VERSION_GREATER_EQUAL "4.0")
  if(NOT DEFINED CMAKE_POLICY_VERSION_MINIMUM OR
      CMAKE_POLICY_VERSION_MINIMUM VERSION_LESS "3.10")
    set(CMAKE_POLICY_VERSION_MINIMUM "3.10")
  endif()
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# find package
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
set(IGN_MSGS_VER 8)
find_package(ignition-transport11 REQUIRED)
set(IGN_TRANSPORT_VER 11)
find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER 6)


# for plugins MecanumDrive2
add_library(MecanumDrive2 SHARED plugins/mecanum_drive2/MecanumDrive2.cc)

target_link_libraries(MecanumDrive2
  ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
)

# for plugins ProjectileShooter
add_library(OmniDrive SHARED plugins/mecanum_drive2/OmniDrive.cc)
target_compile_features(OmniDrive PRIVATE cxx_std_17)
target_link_libraries(OmniDrive Eigen3::Eigen
  ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER})

# for plugins ProjectileShooter
add_library(ProjectileShooter SHARED plugins/projectile_shooter/ProjectileShooter.cc)

target_link_libraries(ProjectileShooter
  ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
)

# for plugins LightBarController
add_library(LightBarController SHARED plugins/light_bar_controller/LightBarController.cc)
target_link_libraries(LightBarController
  ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
)


# Install plugins
install(TARGETS MecanumDrive2 OmniDrive ProjectileShooter LightBarController
    DESTINATION plugins
)

#environment
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo.dsv.in")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_omni_kinematics test/test_omni_kinematics.cpp)
  target_link_libraries(test_omni_kinematics Eigen3::Eigen)
  # no lint test
  # find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()
