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
	# MATH
	# ==========================

	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/math/vec2.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/math/rotation.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/math/transform.hpp

	${CMAKE_CURRENT_SOURCE_DIR}/include/box2dpp/math.hpp
	
)

set(
	BPP_PRIVATE_FILES
	
	# ==========================
	# MATH
	# ==========================

	${CMAKE_CURRENT_SOURCE_DIR}/src/math/vec2.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/math/rotation.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/math/transform.cpp
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
