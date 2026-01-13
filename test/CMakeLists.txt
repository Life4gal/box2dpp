CPMAddPackage(
		NAME PrometheusUT
		GIT_TAG v1.1.0
		GITHUB_REPOSITORY "Life4gal/ut"
		OPTIONS 
		"PROMETHEUS_UT_TEST OFF"
		"PROMETHEUS_UT_INSTALL OFF"
)

project(
	BPP_TEST
	LANGUAGES CXX
)

add_executable(
	${PROJECT_NAME}

	${PROJECT_SOURCE_DIR}/vec2.cpp
	${PROJECT_SOURCE_DIR}/rotation.cpp
	${PROJECT_SOURCE_DIR}/transform.cpp

	${PROJECT_SOURCE_DIR}/main.cpp
)

target_compile_features(
		${PROJECT_NAME}
		PUBLIC
		cxx_std_23
)

target_link_libraries(
		${PROJECT_NAME}
		PRIVATE
		box2dpp
		PrometheusUT
)

add_test(
		NAME ${PROJECT_NAME}
		COMMAND ${PROJECT_NAME}
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)
