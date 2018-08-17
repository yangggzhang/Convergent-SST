cmake_minimum_required(VERSION 2.8.4)
project(SparseRRT)

find_package(OpenRAVE 0.9 REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

add_definitions(-O3 )
add_definitions(-fopenmp)

include_directories(${OpenRAVE_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
include_directories("${PROJECT_SOURCE_DIR}/include")

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

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

set(ODE_VERSION 0.13)
set(PACKAGE_VERSION ${ODE_VERSION})

set(PACKAGE_VERSION_EXACT False)
set(PACKAGE_VERSION_COMPATIBLE False)

if(PACKAGE_FIND_VERSION VERSION_EQUAL PACKAGE_VERSION)
  set(PACKAGE_VERSION_EXACT True)
  set(PACKAGE_VERSION_COMPATIBLE True)
endif()

if(PACKAGE_FIND_VERSION VERSION_LESS PACKAGE_VERSION)
  set(PACKAGE_VERSION_COMPATIBLE True)
endif()

set(ODE_VERSION 0.13)
set(ODE_VERSION_MAJOR 0)
set(ODE_VERSION_MINOR 12)

include(FindPkgConfig)
pkg_check_modules(ODE REQUIRED ode)
if(ODE_FOUND)
	message(STATUS "  ODE_INCLUDE_DIRS= ${ODE_INCLUDE_DIRS}") 
	message(STATUS "  ODE_LIBRARIES= ${ODE_LIBRARIES}") 
	message(STATUS "  ODE_LIBRARY_DIRS= ${ODE_LIBRARY_DIRS}")
	message(STATUS "  ODE_CFLAGS= ${ODE_CFLAGS}")
endif()

if(${ODE_FIND_COMPONENTS} MATCHES drawstuff)
	pkg_check_modules(DS REQUIRED drawstuff)
	if(DS_FOUND)
		message(STATUS "  DS_INCLUDE_DIRS= ${DS_INCLUDE_DIRS}") 
		message(STATUS "  DS_LIBRARIES= ${DS_LIBRARIES}") 
		message(STATUS "  DS_LIBRARY_DIRS= ${DS_LIBRARY_DIRS}")
		message(STATUS "  DS_CFLAGS= ${DS_CFLAGS}")
		include(FindGLUT)
		include(FindOpenGL)
	endif()
endif()

link_directories(${ODE_LIBRARY_DIRS} ${DS_LIBRARY_DIRS})
include_directories(${ODE_INCLUDE_DIRS} ${DS_INCLUDE_DIRS})
add_definitions(${ODE_CFLAGS} ${DS_CFLAGS})
set(ODE_LIBS ${ODE_LIBRARIES} ${DS_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES})


file(GLOB_RECURSE SRC_LIB ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/include/*.hpp)

add_library(${PROJECT_NAME} SHARED ${SRC_LIB})
target_link_libraries(${PROJECT_NAME} ${OpenRAVE_CORE_LIBRARIES} ${Boost_LIBRARIES} ${ODE_LIBRARIES})

file(GLOB files ${PROJECT_SOURCE_DIR}/tests/*.cpp)

foreach(f ${files})
GET_FILENAME_COMPONENT(fname ${f} NAME_WE )
add_executable(${fname} ${f} )
set_target_properties(${fname} PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(${fname} ${PROJECT_NAME} ${OpenRAVE_CORE_LIBRARIES} ${Boost_LIBRARIES} ${ODE_LIBRARIES})
endforeach()