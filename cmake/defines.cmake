# ===================================================================================================
# PLATFORM

if (CMAKE_SYSTEM_NAME STREQUAL "Windows")
	set(
		BPP_PLATFORM_WINDOWS 
		ON
		CACHE BOOL
		""
	)
	# ==> macro.hpp
	set(BPP_PLATFORM_NAME BPP_PLATFORM_WINDOWS)
elseif (CMAKE_SYSTEM_NAME STREQUAL "Linux")
	set(
		BPP_PLATFORM_LINUX 
		ON
		CACHE BOOL
		""
	)
	# ==> macro.hpp
	set(BPP_PLATFORM_NAME BPP_PLATFORM_LINUX)
elseif (CMAKE_SYSTEM_NAME STREQUAL "Darwin")
	set(
		BPP_PLATFORM_DARWIN 
		ON
		CACHE BOOL
		""
	)
	# ==> macro.hpp
	set(BPP_PLATFORM_NAME BPP_PLATFORM_DARWIN)
else ()
	message(FATAL_ERROR "[BPP] Unknown Platform: ${CMAKE_SYSTEM_NAME}")
endif (CMAKE_SYSTEM_NAME STREQUAL "Windows")

# ===================================================================================================
# ARCHITECTURE

if(CMAKE_SYSTEM_PROCESSOR)
	string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" PROC_LOWER)
	
	if(PROC_LOWER MATCHES "amd64|x86_64|x64|win64")
		set(
			BPP_ARCH_X64 
			ON
			CACHE BOOL
			""
		)
		# ==> macro.hpp
		set(BPP_ARCH_NAME BPP_ARCH_X64)
	elseif(PROC_LOWER MATCHES "i.86|i86|i686|i586|i486|i386|x86")
		set(
			BPP_ARCH_X86 
			ON
			CACHE BOOL
			""
		)
		# ==> macro.hpp
		set(BPP_ARCH_NAME BPP_ARCH_X86)
	elseif(PROC_LOWER MATCHES "aarch64|arm64|armv8")
		set(
			BPP_ARCH_ARM64 
			ON
			CACHE BOOL
			""
		)
		# ==> macro.hpp
		set(BPP_ARCH_NAME BPP_ARCH_ARM64)
	elseif(PROC_LOWER MATCHES "arm.*")
		set(
			BPP_ARCH_ARM 
			ON
			CACHE BOOL
			""
		)
		# ==> macro.hpp
		set(BPP_ARCH_NAME BPP_ARCH_ARM)
	else()
		set(
			BPP_ARCH_UNKNOWN 
			ON
			CACHE BOOL
			""
		)
		# ==> macro.hpp
		set(BPP_ARCH_NAME BPP_ARCH_UNKNOWN)
	endif()
else()
	set(
		BPP_ARCH_UNKNOWN 
		ON
		CACHE BOOL
		""
	)
	# ==> macro.hpp
	set(BPP_ARCH_NAME BPP_ARCH_UNKNOWN)
endif(CMAKE_SYSTEM_PROCESSOR)

# ===================================================================================================
# COMPILER

if (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
	set(
		BPP_COMPILER_MSVC 
		ON
		CACHE BOOL
		""
	)
	# ==> macro.hpp
	set(BPP_COMPILER_NAME BPP_COMPILER_MSVC)
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
	if (CMAKE_CXX_SIMULATE_ID STREQUAL "MSVC")
		set(
			BPP_COMPILER_CLANG_CL 
			ON
			CACHE BOOL
			""
		)
		# ==> macro.hpp
		set(BPP_COMPILER_NAME BPP_COMPILER_CLANG_CL)
	else ()
		set(
			BPP_COMPILER_CLANG 
			ON
			CACHE BOOL
			""
		)
		# ==> macro.hpp
		set(BPP_COMPILER_NAME BPP_COMPILER_CLANG)
	endif (CMAKE_CXX_SIMULATE_ID STREQUAL "MSVC")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	set(
		BPP_COMPILER_GNU 
		ON
		CACHE BOOL
		""
	)
	# ==> macro.hpp
	set(BPP_COMPILER_NAME BPP_COMPILER_GNU)
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
	set(
		BPP_COMPILER_CLANG_APPLE 
		ON
		CACHE BOOL
		""
	)
	# ==> macro.hpp
	set(BPP_COMPILER_NAME BPP_COMPILER_CLANG_APPLE)
else ()
	message(FATAL_ERROR "[BPP] Unknown compiler: ${CMAKE_CXX_COMPILER}")
endif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")

# ===================================================================================================
# COMPILE FLAGS

if (BPP_COMPILER_MSVC)
	set(
		BPP_COMPILE_FLAGS 
		"/D_CRT_SECURE_NO_WARNINGS"
		"/DNOMINMAX"
		"/DWIN32_LEAN_AND_MEAN"
		"/DVC_EXTRALEAN"
		"/DSTRICT"
		"/utf-8"
		"/W3"
		"/WX"
		"/Zc:preprocessor"
		"/permissive-"
		CACHE STRING
		""
	)
elseif (BPP_COMPILER_CLANG_CL)
	set(
		BPP_COMPILE_FLAGS 
		"/D_CRT_SECURE_NO_WARNINGS"
		"/DNOMINMAX"
		"/DWIN32_LEAN_AND_MEAN"
		"/DVC_EXTRALEAN"
		"/DSTRICT"
		"/utf-8"
		"/W3"
		"/WX"
		"/permissive-"
		CACHE STRING
		""
	)
elseif (BPP_COMPILER_CLANG)
	set(
		BPP_COMPILE_FLAGS 
		"-Wall"
		"-Wextra"
		"-Wpedantic"
		"-Werror"
		"-Wconversion"
		"-Wshadow"
		"-Wold-style-cast"
		"-Wnull-dereference"
		"-Wdouble-promotion"
		CACHE STRING
		""
	)
elseif (BPP_COMPILER_GNU)
	set(
		BPP_COMPILE_FLAGS 
		"-Wall"
		"-Wextra"
		"-Wpedantic"
		"-Werror"
		"-Wconversion"
		"-Wshadow"
		"-Wold-style-cast"
		"-Wnull-dereference"
		"-Wlogical-op"
		"-Wduplicated-cond"
		"-Wduplicated-branches"
		CACHE STRING
		""
	)
elseif (BPP_COMPILER_CLANG_APPLE)
	set(
		BPP_COMPILE_FLAGS 
		"-Wall"
		"-Wextra"
		"-Wpedantic"
		"-Werror"
		"-Wconversion"
	)
endif (BPP_COMPILER_MSVC)

# ===================================================================================================
# GIT

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/git.cmake)

# ===================================================================================================
# OUTPUT INFO

message(STATUS "")
message(STATUS "================================================================")
message(STATUS "  ${PROJECT_NAME} - v${BPP_VERSION}")
message(STATUS "================================================================")
# PLATFORM + ARCHITECTURE + COMPILER + COMPILE FLAGS
message(STATUS "  Platform: ${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "  CMake Version: ${CMAKE_VERSION}")
message(STATUS "  Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  Compile Flags: ${BPP_COMPILE_FLAGS}")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  Build Test: ${CLP_TEST}")
# GIT
if(PROJECT_IS_TOP_LEVEL)
	message(STATUS "  Git: ")
	message(STATUS "      Branch: ${BPP_GIT_BRANCH_NAME}")
	message(STATUS "      Commit: ${BPP_GIT_COMMIT_HASH}${BPP_GIT_DIRTY_FLAG}")
	message(STATUS "      Date: ${BPP_GIT_COMMIT_DATE}")
	message(STATUS "      Tag: ${BPP_GIT_TAG}")
	message(STATUS "      Status: ${BPP_GIT_DIRTY_STATUS}")
	message(STATUS "================================================================")
	message(STATUS "")
else()
endif(PROJECT_IS_TOP_LEVEL)

