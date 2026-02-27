add_library(usermod_pz_fifo INTERFACE)

target_sources(usermod_pz_fifo INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/pz_fifo.c
)

target_include_directories(usermod_pz_fifo INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(usermod INTERFACE usermod_pz_fifo)
