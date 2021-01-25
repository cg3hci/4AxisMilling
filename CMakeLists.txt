cmake_minimum_required(VERSION 3.9)
project(4AxisMilling)


add_subdirectory(lib/MultiLabelOptimization)
add_definitions(-DMULTI_LABEL_OPTIMIZATION_INCLUDED)
add_subdirectory(lib/clipper)
set(CG3_STATIC ON)
add_subdirectory(cg3lib)

#include(${CMAKE_CURRENT_SOURCE_DIR}/cg3lib/cg3.cmake)

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
	${CMAKE_CURRENT_SOURCE_DIR}/GUI/managers/fafmanager.h
	${CMAKE_CURRENT_SOURCE_DIR}/GUI/managers/fafsegmentationmanager.h
	${CMAKE_CURRENT_SOURCE_DIR}/methods/faf/faf_split.h)

set(SOURCES
	${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
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
	${CMAKE_CURRENT_SOURCE_DIR}/GUI/managers/fafmanager.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/GUI/managers/fafsegmentationmanager.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/methods/faf/faf_split.cpp)


set(FORMS
	${CMAKE_CURRENT_SOURCE_DIR}/GUI/managers/fafsegmentationmanager.ui
	${CMAKE_CURRENT_SOURCE_DIR}/GUI/managers/fafmanager.ui)

set(RESOURCES
	${CMAKE_CURRENT_SOURCE_DIR}/resources/resources.qrc)

add_executable(
	fourAxisMilling
	${HEADERS} ${SOURCES} ${FORMS} ${RESOURCES})

target_link_libraries(
	fourAxisMilling
	PUBLIC 
		cg3lib gco clipper
	)
