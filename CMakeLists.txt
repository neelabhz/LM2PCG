cmake_minimum_required(VERSION 3.16)
project(indoor_pc_pipeline CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find PCL
find_package(PCL 1.10 REQUIRED COMPONENTS common io kdtree)

add_library(pcg
    src/geometry/clustering.cpp
    src/geometry/bbox.cpp
    src/geometry/volume.cpp
    src/io/ply_io.cpp
    src/io/csv_writer.cpp
    src/io/params.cpp
    src/recon/poisson.cpp
    src/recon/af.cpp
)

target_include_directories(pcg PUBLIC include ${PCL_INCLUDE_DIRS})

# On macOS, suppress some PCL deprecation warnings
if(APPLE)
    target_compile_definitions(pcg PRIVATE _GLIBCXX_USE_CXX11_ABI=1)
endif()

# PCL definitions and libraries
add_definitions(${PCL_DEFINITIONS})
if(PCL_FOUND)
    message(STATUS "PCL found: ${PCL_VERSION}")
endif()

target_link_libraries(pcg PUBLIC ${PCL_LIBRARIES})

add_executable(pcg_room src/apps/pcg_room.cpp)

target_link_libraries(pcg_room PRIVATE pcg ${PCL_LIBRARIES})

target_include_directories(pcg_room PRIVATE include ${PCL_INCLUDE_DIRS})

# Optional: CGAL for reconstruction app
find_package(CGAL QUIET COMPONENTS Core)
if(CGAL_FOUND)
    message(STATUS "CGAL found: building pcg_reconstruct")
    add_executable(pcg_reconstruct src/apps/pcg_reconstruct.cpp)
    target_include_directories(pcg_reconstruct PRIVATE include ${PCL_INCLUDE_DIRS})
    target_link_libraries(pcg_reconstruct PRIVATE pcg ${PCL_LIBRARIES} CGAL::CGAL)

    # Mesh volume CLI
    add_executable(pcg_volume src/apps/pcg_volume.cpp)
    target_include_directories(pcg_volume PRIVATE include ${PCL_INCLUDE_DIRS})
    target_link_libraries(pcg_volume PRIVATE pcg ${PCL_LIBRARIES} CGAL::CGAL)
else()
    message(STATUS "CGAL not found: pcg_reconstruct will not be built")
endif()

# Installation rules (optional)
install(TARGETS pcg pcg_room RUNTIME DESTINATION bin LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
