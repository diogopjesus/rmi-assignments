
set(agent_SRCS
    # Source
    controller.cpp
    map.cpp
    pose.cpp
    utils.cpp
    # Header
    ${CMAKE_SOURCE_DIR}/include/agent/controller.h
    ${CMAKE_SOURCE_DIR}/include/agent/map.h
    ${CMAKE_SOURCE_DIR}/include/agent/pose.h
    ${CMAKE_SOURCE_DIR}/include/agent/utils.h
)

add_library(agent SHARED ${agent_SRCS})

target_include_directories(agent PUBLIC
                            "${CMAKE_CURRENT_SOURCE_DIR}"
                            "${CMAKE_SOURCE_DIR}/include"
                            #"${CMAKE_SOURCE_DIR}/src"
                        )

target_link_libraries(agent robSock)
