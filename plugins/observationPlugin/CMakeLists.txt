ADD_OBSERVATION_PLUGIN(movoObservationPlugin ${CMAKE_CURRENT_LIST_DIR}/MovoObservationPlugin.cpp)
target_include_directories(movoObservationPlugin PRIVATE "${ROOT_PATH}/plugins/shared/")
