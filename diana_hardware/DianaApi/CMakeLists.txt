# DianaApi CMake Wrapper. Created by: Yang Luo
cmake_minimum_required(VERSION 3.10)

project(libDianaApi)

add_library(${PROJECT_NAME} SHARED IMPORTED GLOBAL)
set_property(TARGET ${PROJECT_NAME} PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/libDianaApi.so)
set_property(TARGET ${PROJECT_NAME} PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_directories(${PROJECT_NAME} INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/bin
)

target_link_libraries(${PROJECT_NAME} INTERFACE
        DianaApi
        BasicSdk
        GenericAlgorithm
        ToolSdk
        VersionApi
        xml2
)

install(
        DIRECTORY bin/
        DESTINATION lib
)