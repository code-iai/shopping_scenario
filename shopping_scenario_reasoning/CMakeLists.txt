cmake_minimum_required(VERSION 2.8.3)
project(shopping_scenario_reasoning)

find_package(catkin REQUIRED shopping_scenario_models)

catkin_package(
  DEPENDS shopping_scenario_models
)
