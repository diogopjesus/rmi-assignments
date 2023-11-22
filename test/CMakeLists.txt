find_package(Catch2 3 REQUIRED)

set(tests_SRCS 
    TestPerceivedMap.cpp
)

# These tests can use the Catch2-provided main
add_executable(tests ${tests_SRCS})

target_include_directories(tests PRIVATE
                            "${CMAKE_CURRENT_SOURCE_DIR}"
                            "${CMAKE_SOURCE_DIR}/src"
                            "${CMAKE_SOURCE_DIR}/include"
                        )

target_link_libraries(tests PRIVATE Catch2::Catch2WithMain agent)
