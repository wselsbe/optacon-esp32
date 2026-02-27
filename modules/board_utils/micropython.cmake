add_library(usermod_board_utils INTERFACE)

target_sources(usermod_board_utils INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/board_utils.c
)

target_include_directories(usermod_board_utils INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(usermod INTERFACE usermod_board_utils)
