cmake_minimum_required(VERSION 3.25)

set(
	BPP_MAJOR_VERSION 
	1
	CACHE STRING 
	"Box2dpp major version"
)
set(
	BPP_MINOR_VERSION 
	0
	CACHE STRING 
	"Box2dpp minor version"
)
set(
	BPP_PATCH_VERSION 
	0 
	CACHE STRING 
	"Box2dpp patch version"
)
set(
	BPP_VERSION 
	"${BPP_MAJOR_VERSION}.${BPP_MINOR_VERSION}.${BPP_PATCH_VERSION}" 
	CACHE STRING 
	"Box2dpp version"
)

project(
		box2dpp
		VERSION ${BPP_VERSION}
		DESCRIPTION "box2dpp"
		HOMEPAGE_URL "https://github.com/Life4gal/box2dpp"
		LANGUAGES CXX
)

# ===================================================================================================
# OPTION

option(BPP_TEST "Generate the test target." ON)

# ===================================================================================================
# DEFINES

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/defines.cmake)

# ===================================================================================================
# DEPENDENCIES

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/find_cpm.cmake)

# GLM
#find_package(GLM REQUIRED)

# ===================================================================================================
# PACK

set(BPP_VERSION_HEADER_FILE ${CMAKE_CURRENT_BINARY_DIR}/include/box2dpp/version.hpp)

configure_file(
	${CMAKE_CURRENT_SOURCE_DIR}/cmake/version.hpp.in
	${BPP_VERSION_HEADER_FILE}
	@ONLY
)

# ===================================================================================================
# LIBRARY

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
#set(CMAKE_POSITION_INDEPENDENT_CODE ON)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_library(${PROJECT_NAME})

set_target_properties(
	${PROJECT_NAME} PROPERTIES
	VERSION ${BPP_VERSION}
	SOVERSION ${BPP_MAJOR_VERSION}
)

target_include_directories(
	${PROJECT_NAME} 

	INTERFACE 
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	# version header file
	$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

target_compile_options(
	${PROJECT_NAME}
	PUBLIC
	
	${BPP_COMPILE_FLAGS}
)

#target_compile_definitions(
#	${PROJECT_NAME}
#	PUBLIC
#
#)

target_compile_features(
	${PROJECT_NAME}
	PRIVATE
	cxx_std_23
)

#target_link_libraries(
#    ${PROJECT_NAME} 
#    PUBLIC 
#
#    glm::glm
#)

set(
	BPP_PUBLIC_FILES 
	
	# ==========================
	# UTILITY
	# ==========================

	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/utility/value_cast.hpp

	# ==========================
	# MATH
	# ==========================

	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/math/vec2.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/math/rotation.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/math/transform.hpp

	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/math.hpp
		
	# ==========================
	# SHAPE
	# ==========================

	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/shape/circle.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/shape/capsule.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/shape/polygon.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/shape/segment.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/shape/aabb.hpp

	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/shape/mass_data.hpp

	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/shape.hpp

	# ==========================
	# COLLISION
	# ==========================

	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/collision/shape_proxy.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/collision/simplex.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/collision/distance.hpp

	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/collision/ray_cast.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/collision/shape_cast.hpp

	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/collision/time_of_impact.hpp

	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/collision.hpp
	
	# ==========================
	# DESC
	# ==========================

	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/desc/id.hpp

	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/desc/world.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/desc/body.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/desc/filter.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/desc/surface.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/desc/shape.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/desc/chain.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/desc/joint.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/desc/explosion.hpp

	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/desc.hpp

	# ==========================
	# EVENT
	# ==========================

	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/event/sensor.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/event/contact.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/event/body.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/event/joint.hpp

	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/event.hpp

	# ==========================
	# MAIN
	# ==========================

	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/box2dpp.hpp
)

set(
	BPP_PRIVATE_FILES
	
	# ==========================
	# MATH
	# ==========================

	${CMAKE_CURRENT_SOURCE_DIR}/src/math/vec2.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/math/rotation.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/math/transform.cpp
			
	# ==========================
	# SHAPE
	# ==========================
	
	${CMAKE_CURRENT_SOURCE_DIR}/src/shape/circle.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/shape/capsule.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/shape/polygon.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/shape/segment.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/shape/aabb.cpp

	${CMAKE_CURRENT_SOURCE_DIR}/src/shape/mass_data.cpp

	# ==========================
	# COLLISION
	# ==========================

	${CMAKE_CURRENT_SOURCE_DIR}/src/collision/shape_proxy.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/collision/simplex.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/collision/distance.cpp

	${CMAKE_CURRENT_SOURCE_DIR}/src/collision/ray_cast.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/collision/shape_cast.cpp

	${CMAKE_CURRENT_SOURCE_DIR}/src/collision/time_of_impact.cpp

	# ==========================
	# DESC
	# ==========================

	${CMAKE_CURRENT_SOURCE_DIR}/src/desc/world.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/desc/body.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/desc/filter.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/desc/surface.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/desc/shape.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/desc/chain.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/desc/joint.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/desc/explosion.cpp

	# ==========================
	# MAIN
	# ==========================

	${CMAKE_CURRENT_SOURCE_DIR}/src/box2dpp.cpp
)

set_source_files_properties(
	${BPP_PUBLIC_FILES}
	${BPP_PRIVATE_FILES}
	PROPERTIES
	LANGUAGE CXX
)

target_sources(
	${PROJECT_NAME}
	PUBLIC
	FILE_SET public_header_files
	TYPE HEADERS
	BASE_DIRS 
	${CMAKE_CURRENT_SOURCE_DIR}/include
	# version header file
	${CMAKE_CURRENT_BINARY_DIR}/include
	FILES
	${BPP_PUBLIC_FILES}
	${BPP_VERSION_HEADER_FILE}
)

target_sources(
	${PROJECT_NAME}
	PRIVATE
	${BPP_PRIVATE_FILES}
)

# ===================================================================================================
# TESTS

if (BPP_TEST)
	enable_testing()
	add_subdirectory(test)
endif (BPP_TEST)

# ===================================================================================================
# INSTALL
