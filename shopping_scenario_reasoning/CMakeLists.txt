cmake_minimum_required(VERSION 2.8.3)
project(shopping_scenario_reasoning)

find_package(catkin REQUIRED rosjava_build_tools knowrob_common)

catkin_rosjava_setup(installApp publishMavenJavaPublicationToMavenRepository writeClasspath)

catkin_package(
  DEPENDS shopping_scenario_models knowrob_common
)
