cmake_minimum_required(VERSION 3.5)
project(jaka_kargo_isaacsim)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY
  scripts
  launch
  config
  DESTINATION share/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/run_isaacsim.py
  scripts/open_isaacsim_stage.py
  scripts/isaacsim_moveit.py
  scripts/python.sh
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES scripts/assembly_scene.py scripts/assembly_bridge.py scripts/assembly_startup.py scripts/assembly_physics_diagnostics.py
  DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(assembly_contract test/test_assembly_contract.py)
  ament_add_pytest_test(assembly_startup test/test_assembly_startup.py)
endif()
ament_package()