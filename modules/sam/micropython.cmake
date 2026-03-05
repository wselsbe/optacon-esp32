# modules/sam/micropython.cmake — SAM speech synthesizer C module (ctoth/SAM fork)
add_library(usermod_sam INTERFACE)

target_sources(usermod_sam INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/mod_sam.c
    ${CMAKE_CURRENT_LIST_DIR}/sam.c
    ${CMAKE_CURRENT_LIST_DIR}/sam_tabs.c
    ${CMAKE_CURRENT_LIST_DIR}/render.c
    ${CMAKE_CURRENT_LIST_DIR}/reciter.c
    ${CMAKE_CURRENT_LIST_DIR}/phoneme_parser.c
    ${CMAKE_CURRENT_LIST_DIR}/rules.c
    ${CMAKE_CURRENT_LIST_DIR}/transitions.c
)

target_include_directories(usermod_sam INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../pz_drive
)

target_link_libraries(usermod_sam INTERFACE usermod_pz_drive)
target_link_libraries(usermod INTERFACE usermod_sam)
