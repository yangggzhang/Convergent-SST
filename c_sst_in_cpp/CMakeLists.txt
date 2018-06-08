cmake_minimum_required(VERSION 2.8.4)
project(SparseRRT)


find_package( CGAL QUIET COMPONENTS  )

if ( NOT CGAL_FOUND )

  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()  

endif()

# include helper file
include( ${CGAL_USE_FILE} )


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

add_definitions(-O3 )

find_package(Boost COMPONENTS program_options REQUIRED)

include_directories(${Boost_INCLUDE_DIR})
include_directories("${PROJECT_SOURCE_DIR}/include")


file(GLOB_RECURSE SRC_LIB ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/include/*.hpp)

add_library(${PROJECT_NAME} SHARED ${SRC_LIB})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} )

file(GLOB files ${PROJECT_SOURCE_DIR}/tests/*.cpp)

foreach(f ${files})
GET_FILENAME_COMPONENT(fname ${f} NAME_WE )
add_executable(${fname} ${f} )
target_link_libraries(${fname} ${PROJECT_NAME} )
endforeach()
