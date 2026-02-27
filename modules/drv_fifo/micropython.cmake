add_library(usermod_drv_fifo INTERFACE)

target_sources(usermod_drv_fifo INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/drv_fifo.c
)

target_include_directories(usermod_drv_fifo INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(usermod INTERFACE usermod_drv_fifo)
