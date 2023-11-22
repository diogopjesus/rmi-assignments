
set(mainRobot_SRCS
    # Source
    main.cpp
)

add_subdirectory(robSock)
add_subdirectory(agent)

add_executable(mainRobC4 ${mainRobot_SRCS})

target_include_directories(mainRobC4 PRIVATE
                            "${CMAKE_CURRENT_SOURCE_DIR}"
                            "${CMAKE_SOURCE_DIR}/include"
                            #"${CMAKE_SOURCE_DIR}/src"
                        )

target_link_libraries(mainRobC4 agent robSock)

set_target_properties(mainRobC4 PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
