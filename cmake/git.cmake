find_package(Git QUIET)
if(NOT GIT_FOUND)
	message(WARNING "Git not found - version information will not be available")
	set(GIT_AVAILABLE FALSE)
else()
	execute_process(
		COMMAND ${GIT_EXECUTABLE} rev-parse --is-inside-work-tree
		WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		OUTPUT_VARIABLE IS_GIT_REPO
		OUTPUT_STRIP_TRAILING_WHITESPACE
		ERROR_QUIET
		RESULT_VARIABLE GIT_REPO_CHECK_RESULT
	)
	
	if(NOT GIT_REPO_CHECK_RESULT EQUAL "0")
		message(WARNING "Not a Git repository - version information will not be available")
		set(GIT_AVAILABLE FALSE)
	else()
		set(GIT_AVAILABLE TRUE)
	endif()
endif()

if(NOT GIT_AVAILABLE)
	set(BPP_GIT_BRANCH_NAME "unknown")
	set(BPP_GIT_COMMIT_HASH "unknown")
	set(BPP_GIT_COMMIT_DATE "unknown")
	set(BPP_GIT_DIRTY_FLAG "unknown")
	set(BPP_GIT_TAG "unknown")
	set(BPP_GIT_COMMIT_INFO "unknown")
else()
	# ============================================
	# 1. Get branch name
	# ============================================
	execute_process(
		COMMAND ${GIT_EXECUTABLE} symbolic-ref --short HEAD
		WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		OUTPUT_VARIABLE BPP_GIT_BRANCH_NAME
		OUTPUT_STRIP_TRAILING_WHITESPACE
		ERROR_QUIET
		RESULT_VARIABLE BRANCH_RESULT
	)
	
	if(NOT BRANCH_RESULT EQUAL "0")
		execute_process(
			COMMAND ${GIT_EXECUTABLE} describe --contains --all HEAD
			WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
			OUTPUT_VARIABLE BPP_GIT_BRANCH_NAME
			OUTPUT_STRIP_TRAILING_WHITESPACE
			ERROR_QUIET
		)
	endif()
	
	if(NOT BPP_GIT_BRANCH_NAME)
		set(BPP_GIT_BRANCH_NAME "detached")
	endif()
	
	# ============================================
	# 2. Commit Hash
	# ============================================
	execute_process(
		COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
		WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		OUTPUT_VARIABLE BPP_GIT_COMMIT_HASH
		OUTPUT_STRIP_TRAILING_WHITESPACE
		ERROR_QUIET
		RESULT_VARIABLE HASH_RESULT
	)

	if(HASH_RESULT EQUAL "0")
		set(HAS_HASH_INFO 1)
	else()
		set(HAS_HASH_INFO 0)
	endif()
	
	# ============================================
	# 3. Commit Date
	# ============================================
	if(HAS_HASH_INFO)
		execute_process(
			COMMAND ${GIT_EXECUTABLE} log -1 --format=%cd --date=iso-strict
			WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
			OUTPUT_VARIABLE BPP_GIT_COMMIT_DATE
			OUTPUT_STRIP_TRAILING_WHITESPACE
			ERROR_QUIET
		)
	else()
		set(BPP_GIT_COMMIT_DATE "1970-01-01T00:00:00+00:00")
	endif()

	
	# ============================================
	# 4. Dirty flag
	# ============================================
	if(HAS_HASH_INFO)
		execute_process(
			COMMAND ${GIT_EXECUTABLE} diff-index --quiet HEAD --
			WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
			RESULT_VARIABLE BPP_GIT_DIRTY
			ERROR_QUIET
		)
	
		if(BPP_GIT_DIRTY EQUAL "0")
			set(BPP_GIT_DIRTY_FLAG "")
			set(BPP_GIT_DIRTY_STATUS "clean")
		else()
			set(BPP_GIT_DIRTY_FLAG "-dirty")
			set(BPP_GIT_DIRTY_STATUS "dirty")
		endif()
	else()
		set(BPP_GIT_DIRTY_FLAG "")
		set(BPP_GIT_DIRTY_STATUS "unknown")
	endif()

	# ============================================
	# 5. TAG
	# ============================================
	if(HAS_HASH_INFO)
		execute_process(
			COMMAND ${GIT_EXECUTABLE} describe --tags --exact-match
			WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
			OUTPUT_VARIABLE BPP_GIT_TAG
			OUTPUT_STRIP_TRAILING_WHITESPACE
			ERROR_QUIET
		)
	
		if(NOT BPP_GIT_TAG)
			set(BPP_GIT_TAG "none")
		endif()
	else()
		set(BPP_GIT_TAG "none")
	endif()
	
	# ============================================
	# 6. Remote URL
	# ============================================
	execute_process(
		COMMAND ${GIT_EXECUTABLE} config --get remote.origin.url
		WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		OUTPUT_VARIABLE BPP_GIT_REMOTE_URL
		OUTPUT_STRIP_TRAILING_WHITESPACE
		ERROR_QUIET
	)
	
	# ============================================
	# 7. Commit Info
	# ============================================
	if(NOT HAS_HASH_INFO)
		set(BPP_GIT_COMMIT_INFO "empty repository")
		set(BPP_GIT_COMMIT_INFO_SHORT "empty")
		set(BPP_GIT_COMMIT_INFO_FULL "empty repository")
	else()
		string(REPLACE "T" " " DATE_WITHOUT_T "${BPP_GIT_COMMIT_DATE}")
		string(REGEX REPLACE "\\+.*$" "" DATE_SHORT "${DATE_WITHOUT_T}")
		
		set(
			BPP_GIT_COMMIT_INFO_SHORT 
			"${BPP_GIT_BRANCH_NAME}/${BPP_GIT_COMMIT_HASH}${BPP_GIT_DIRTY_FLAG}"
		)
		
		set(
			BPP_GIT_COMMIT_INFO 
			"${BPP_GIT_BRANCH_NAME}/${BPP_GIT_COMMIT_HASH}${BPP_GIT_DIRTY_FLAG} (${DATE_SHORT})"
		)
		
		set(
			BPP_GIT_COMMIT_INFO_FULL 
			"branch:${BPP_GIT_BRANCH_NAME} hash:${BPP_GIT_COMMIT_HASH}${BPP_GIT_DIRTY_FLAG} date:${BPP_GIT_COMMIT_DATE} tag:${BPP_GIT_TAG}"
		)
	endif()
endif()

set(
	BPP_GIT_BRANCH_NAME 
	"${BPP_GIT_BRANCH_NAME}" 
	CACHE STRING "${PROJECT_NAME} git branch name"
)
set(
	BPP_GIT_COMMIT_HASH 
	"${BPP_GIT_COMMIT_HASH}" 
	CACHE STRING "${PROJECT_NAME} git commit hash"
)
set(
	BPP_GIT_COMMIT_DATE 
	"${BPP_GIT_COMMIT_DATE}" 
	CACHE STRING "${PROJECT_NAME} git commit date"
)
set(
	BPP_GIT_DIRTY_FLAG 
	"${BPP_GIT_DIRTY_FLAG}" 
	CACHE STRING "${PROJECT_NAME} git dirty flag"
)
set(
	BPP_GIT_DIRTY_STATUS 
	"${BPP_GIT_DIRTY_STATUS}" 
	CACHE STRING "${PROJECT_NAME} git dirty status"
)
set(
	BPP_GIT_TAG 
	"${BPP_GIT_TAG}" 
	CACHE STRING "${PROJECT_NAME} git tag"
)
set(
	BPP_GIT_REMOTE_URL 
	"${BPP_GIT_REMOTE_URL}" 
	CACHE STRING "${PROJECT_NAME} git remote url"
)
set(
	BPP_GIT_COMMIT_INFO 
	"${BPP_GIT_COMMIT_INFO}" 
	CACHE STRING "${PROJECT_NAME} git commit info"
)
set(
	BPP_GIT_COMMIT_INFO_SHORT 
	"${BPP_GIT_COMMIT_INFO_SHORT}" 
	CACHE STRING "${PROJECT_NAME} git commit info (short)"
)
set(
	BPP_GIT_COMMIT_INFO_FULL 
	"${BPP_GIT_COMMIT_INFO_FULL}" 
	CACHE STRING "${PROJECT_NAME} git commit info (full)"
)
