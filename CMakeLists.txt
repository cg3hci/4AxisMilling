cmake_minimum_required(VERSION 3.13)
project(4AxisMilling)

option(BUILD_4_AXIS_MILLING_GUI "Build an application that allows to control parameters and view result" OFF)
option(BUILD_4_AXIS_MILLING_CLI "Build a CLI application to run the algorithm from command line" ON)

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()

add_subdirectory(lib/MultiLabelOptimization)
add_subdirectory(lib/clipper)

set(CG3_STATIC ON)
if (BUILD_4_AXIS_MILLING_GUI)
	set(CG3_VIEWER ON)
endif()
set(CG3_CGAL ON)
set(CG3_LIBIGL ON)
set(CG3_VCGLIB ON)
add_subdirectory(cg3lib)

if (BUILD_4_AXIS_MILLING_CLI)
	#c++17 required for cli
	set(CMAKE_CXX_STANDARD 17)
	set(CMAKE_CXX_STANDARD_REQUIRED ON)
	set(CMAKE_CXX_EXTENSIONS OFF)
endif()

find_package(Qt5 COMPONENTS Widgets)

set(HEADERS
	${CMAKE_CURRENT_SOURCE_DIR}/methods/faf/faf_details.h
	${CMAKE_CURRENT_SOURCE_DIR}/methods/faf/faf_smoothlines.h
	${CMAKE_CURRENT_SOURCE_DIR}/methods/fouraxisfabrication.h
	${CMAKE_CURRENT_SOURCE_DIR}/methods/faf/faf_data.h
	${CMAKE_CURRENT_SOURCE_DIR}/methods/faf/faf_optimalrotation.h
	${CMAKE_CURRENT_SOURCE_DIR}/methods/faf/faf_extremes.h
	${CMAKE_CURRENT_SOURCE_DIR}/methods/faf/faf_visibilitycheck.h
	${CMAKE_CURRENT_SOURCE_DIR}/methods/faf/faf_frequencies.h
	${CMAKE_CURRENT_SOURCE_DIR}/methods/faf/faf_extraction.h
	${CMAKE_CURRENT_SOURCE_DIR}/methods/faf/faf_charts.h
	${CMAKE_CURRENT_SOURCE_DIR}/methods/faf/faf_association.h
	${CMAKE_CURRENT_SOURCE_DIR}/methods/faf/includes/view_renderer.h
	${CMAKE_CURRENT_SOURCE_DIR}/methods/faf/faf_optimization.h
	${CMAKE_CURRENT_SOURCE_DIR}/methods/faf/faf_smoothing.h
	${CMAKE_CURRENT_SOURCE_DIR}/methods/faf/faf_various.h
	${CMAKE_CURRENT_SOURCE_DIR}/methods/faf/faf_split.h)

set(HEADERS_GUI
	${CMAKE_CURRENT_SOURCE_DIR}/GUI/managers/fafmanager.h
	${CMAKE_CURRENT_SOURCE_DIR}/GUI/managers/fafsegmentationmanager.h)

set(HEADERS_CLI
	${CMAKE_CURRENT_SOURCE_DIR}/faf_parameters.h
	${CMAKE_CURRENT_SOURCE_DIR}/faf_pipeline.h)

set(SOURCES
	${CMAKE_CURRENT_SOURCE_DIR}/methods/faf/faf_data.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/methods/faf/faf_details.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/methods/faf/faf_optimalrotation.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/methods/faf/faf_extremes.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/methods/faf/faf_smoothlines.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/methods/faf/faf_visibilitycheck.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/methods/faf/faf_frequencies.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/methods/faf/faf_extraction.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/methods/faf/faf_charts.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/methods/faf/faf_association.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/methods/faf/includes/view_renderer.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/methods/faf/faf_optimization.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/methods/faf/faf_smoothing.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/methods/faf/faf_various.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/methods/faf/faf_split.cpp)

set(SOURCES_CLI
	${CMAKE_CURRENT_SOURCE_DIR}/faf_pipeline.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/main_cli.cpp)

set(SOURCES_GUI
	${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/GUI/managers/fafmanager.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/GUI/managers/fafsegmentationmanager.cpp)

set(FORMS
	${CMAKE_CURRENT_SOURCE_DIR}/GUI/managers/fafsegmentationmanager.ui
	${CMAKE_CURRENT_SOURCE_DIR}/GUI/managers/fafmanager.ui)

set(RESOURCES
	${CMAKE_CURRENT_SOURCE_DIR}/resources/resources.qrc)

if (BUILD_4_AXIS_MILLING_GUI)
	add_executable(
		fourAxisMillingGui
		${HEADERS} ${HEADERS_GUI} ${SOURCES} ${SOURCES_GUI} ${FORMS} ${RESOURCES})
	
	target_compile_definitions(
		fourAxisMillingGui
		PRIVATE
			MULTI_LABEL_OPTIMIZATION_INCLUDED)
	
	target_link_libraries(
		fourAxisMillingGui
		PUBLIC 
			cg3lib gco clipper
		)
endif()

if (BUILD_4_AXIS_MILLING_CLI)
	add_executable(
		fourAxisMilling
		${HEADERS} ${HEADERS_CLI} ${SOURCES} ${SOURCES_CLI})
	
	target_compile_definitions(
		fourAxisMilling
		PRIVATE
			MULTI_LABEL_OPTIMIZATION_INCLUDED
			FAF_NO_GL_VISIBILITY)
	
	target_link_libraries(
		fourAxisMilling
		PUBLIC 
			cg3lib gco clipper
		)
endif()
