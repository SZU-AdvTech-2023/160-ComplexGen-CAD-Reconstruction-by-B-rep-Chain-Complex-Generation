cmake_minimum_required(VERSION 3.10)

project(ComplexGenTrim)

find_package(CGAL CONFIG REQUIRED)
find_package(jsoncpp CONFIG REQUIRED)
find_package(tomlplusplus CONFIG REQUIRED)

find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

FILE(GLOB SOURCE_FILE src/*.cpp src/*.h config.toml)
  
# OpenCASCADE Path
set(OpenCASCADE_DIR "C:/dev/OCCT_7_2/cmake")
find_package(OpenCASCADE CONFIG REQUIRED)
FILE(GLOB OpenCASCADE_DLL ${OpenCASCADE_BINARY_DIR}/../bini/*.dll) # Only needed on windows
if(CMAKE_CONFIGURATION_TYPES)
    set(TARGET_OUTPUT_PATH ${PROJECT_BINARY_DIR}/$<CONFIG>)
else()
    set(TARGET_OUTPUT_PATH ${PROJECT_BINARY_DIR})
endif()

add_executable(main ${SOURCE_FILE})
set_target_properties(main PROPERTIES
                CXX_STANDARD 17
                CXX_EXTENSIONS OFF)

target_include_directories(main PRIVATE ${OpenCASCADE_INCLUDE_DIR})

target_link_libraries(main 
    PRIVATE CGAL::CGAL
    JsonCpp::JsonCpp
    tomlplusplus::tomlplusplus
)

target_link_libraries(main PRIVATE ${OpenCASCADE_LIBRARIES})

# Copy OpenCASCADE dlls to output directory
add_custom_command(TARGET main
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy ${OpenCASCADE_DLL} ${TARGET_OUTPUT_PATH}
)
