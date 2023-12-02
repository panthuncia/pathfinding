# CMakeList.txt : CMake project for pathfinding, include source and define
# project specific logic here.
#

# Add source to this project's executable.
add_executable (pathfinding "pathfinding.cpp" "pathfinding.h" "pathfinding_strategy.h" "map.h" "pathfinding_strategy_base.h" "node.h" "map.cpp" "astar_pathfinding_strategy.h" "astar_pathfinding_strategy.cpp"  "utilities.h" "utilities.cpp" "raycast.h" "linear_raycast_pathfinding_strategy.h" "linear_raycast_pathfinding_strategy.cpp" "one_tack_pathfinding_strategy.h" "one_tack_pathfinding_strategy.cpp" "raycast.cpp")
if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET pathfinding PROPERTY CXX_STANDARD 20)
endif()
target_link_libraries(pathfinding PRIVATE ${OpenCV_LIBS})
# TODO: Add tests and install targets if needed.
