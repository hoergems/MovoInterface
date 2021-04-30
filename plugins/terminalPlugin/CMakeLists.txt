ADD_TERMINAL_PLUGIN(movoTerminalPlugin ${CMAKE_CURRENT_LIST_DIR}/MovoTerminalPlugin.cpp)
target_include_directories(movoTerminalPlugin PRIVATE ${ROOT_PATH}/plugins/shared/)
