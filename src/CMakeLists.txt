add_library(swarm_robustness MODULE swarm_robustness.h swarm_robustness.cpp 
obstacle.h 
obstacle.cpp 
data.h)
target_link_libraries(swarm_robustness
  argos3core_simulator
  argos3plugin_simulator_epuck
  argos3plugin_simulator_genericrobot)

