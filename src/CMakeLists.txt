
set(mainRobot_SRCS
    # Source
    mainRob.cpp
    challenges/agentC4.cpp
    # Headers
    challenges/agentC4.h
)

add_subdirectory(robSock)
add_subdirectory(agent)

add_executable(mainRob ${mainRobot_SRCS})

target_include_directories(mainRob PRIVATE
                            "${CMAKE_CURRENT_SOURCE_DIR}"
                            "${CMAKE_SOURCE_DIR}/include"
                            #"${CMAKE_SOURCE_DIR}/src"
                        )

target_link_libraries(mainRob agent robSock)

set_target_properties(mainRob PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
