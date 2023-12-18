find_package(Catch2 3 REQUIRED)

set(test-map_SRCS 
    test-map.cpp
)

# These tests can use the Catch2-provided main
add_executable(test-map ${tests_SRCS})

target_include_directories(test-map PRIVATE
                            "${CMAKE_CURRENT_SOURCE_DIR}"
                            "${CMAKE_SOURCE_DIR}/src"
                            "${CMAKE_SOURCE_DIR}/include"
                        )

target_link_libraries(test-map PRIVATE Catch2::Catch2WithMain agent)

set_target_properties(test-map PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
