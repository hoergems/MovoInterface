ADD_INITIAL_BELIEF_PLUGIN(movoInitialBeliefPlugin ${CMAKE_CURRENT_LIST_DIR}/MovoInitialBeliefPlugin.cpp)
target_include_directories(movoInitialBeliefPlugin PRIVATE ${ROOT_PATH}/plugins/shared/)
