
set(robSock_SRCS
    # Source
    cmeasures.cpp
    croblink.cpp
    csimparam.cpp
    netif.cpp
    RobSock.cpp
    structureparser.cpp
    # Headers
    ${CMAKE_SOURCE_DIR}/include/robSock/cmeasures.h
    ${CMAKE_SOURCE_DIR}/include/robSock/croblink.h
    ${CMAKE_SOURCE_DIR}/include/robSock/csimparam.h
    ${CMAKE_SOURCE_DIR}/include/robSock/netif.h
    ${CMAKE_SOURCE_DIR}/include/robSock/RobSock.h
    ${CMAKE_SOURCE_DIR}/include/robSock/structureparser.h
)

add_library(robSock SHARED ${robSock_SRCS})

target_include_directories(robSock PUBLIC
                            "${CMAKE_CURRENT_SOURCE_DIR}"
                            "${CMAKE_SOURCE_DIR}/include"
                            #"${CMAKE_SOURCE_DIR}/src"
                        )

target_compile_definitions(robSock PRIVATE -DCIBERQTAPP)

target_link_libraries(robSock Qt5::Widgets Qt5::Xml)
