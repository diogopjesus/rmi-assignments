
set(agent_SRCS
    # Source
    agentC4.cpp
    controller.cpp
    localmap.cpp
    localpose.cpp
    # Header
    ${CMAKE_SOURCE_DIR}/include/agent/agentC4.h
    ${CMAKE_SOURCE_DIR}/include/agent/controller.h
    ${CMAKE_SOURCE_DIR}/include/agent/localmap.h
    ${CMAKE_SOURCE_DIR}/include/agent/localpose.h
)

add_library(agent SHARED ${agent_SRCS})

target_include_directories(agent PUBLIC
                            "${CMAKE_CURRENT_SOURCE_DIR}"
                            "${CMAKE_SOURCE_DIR}/include"
                            #"${CMAKE_SOURCE_DIR}/src"
                        )

target_link_libraries(agent robSock)
