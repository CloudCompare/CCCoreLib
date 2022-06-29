# SPDX-License-Identifier: MIT
# Copyright © Andy Maloney <asmaloney@gmail.com>

cmake_minimum_required( VERSION 3.10 )

project( CCCoreLib
	DESCRIPTION
		"Core CloudCompare data structures & algorithms for handling point clouds"
	LANGUAGES
		CXX
	VERSION
		1.0
)

# Options
option( CCCORELIB_USE_CGAL
	"Compile CCCoreLib with CGAL (to enable Delaunay 2.5D triangulation with a GPL-compliant licence)"
	OFF
)
option( CCCORELIB_USE_TBB
	"Compile CCCoreLib with Intel Threading Building Blocks lib (enables some parallel processing )"
	OFF
)
option( CCCORELIB_USE_QT_CONCURRENT
	"Compile CCCoreLib with QtConcurrent (to enable parallel processing)"
	ON
)
option( CCCORELIB_SHARED
	"Compile CCCoreLib as a shared library"
	ON
)
option( CCCORELIB_SCALAR_DOUBLE
	"Define ScalarType as double (instead of float)"
	OFF
)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Add the library (shared or static)
if ( CCCORELIB_SHARED )
	add_library( CCCoreLib SHARED )

	set_target_properties( CCCoreLib
		PROPERTIES
			CXX_VISIBILITY_PRESET hidden
			VISIBILITY_INLINES_HIDDEN 1
	)
else()
	add_library( CCCoreLib STATIC )

	target_compile_definitions( CCCoreLib
		PRIVATE
			CC_CORE_LIB_STATIC_DEFINE
	)
endif()

set_target_properties( CCCoreLib
	PROPERTIES
		DEBUG_POSTFIX d
)

add_library( CCCoreLib::CCCoreLib ALIAS CCCoreLib )

# Generate the export header file
include( GenerateExportHeader )

generate_export_header( CCCoreLib
	EXPORT_FILE_NAME ${CMAKE_CURRENT_BINARY_DIR}/exports/CCCoreLibExport.h
	EXPORT_MACRO_NAME CC_CORE_LIB_API
)

target_sources( CCCoreLib
	PRIVATE
		${CMAKE_CURRENT_BINARY_DIR}/exports/CCCoreLibExport.h
)

install(
	FILES
		${CMAKE_CURRENT_BINARY_DIR}/exports/CCCoreLibExport.h
	DESTINATION
		include/CCCoreLib
)

target_include_directories( CCCoreLib
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/exports>
		$<INSTALL_INTERFACE:include/CCCoreLib>
)

# ccache
# https://crascit.com/2016/04/09/using-ccache-with-cmake/
find_program( CCACHE_PROGRAM ccache )

if ( CCACHE_PROGRAM )
	set_target_properties( CCCoreLib
		PROPERTIES
			CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}"
			C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}"
	)
endif()

# Main sources and includes
add_subdirectory( include )
add_subdirectory( src )

# Compiler & definitions
target_compile_features( CCCoreLib
	PRIVATE
		cxx_std_14
)

set_target_properties( CCCoreLib
	PROPERTIES
		CXX_EXTENSIONS OFF
)

target_compile_definitions( CCCoreLib
	PRIVATE
		"$<$<CONFIG:DEBUG>:CC_DEBUG>"
)

if ( CCCORELIB_SCALAR_DOUBLE )
	target_compile_definitions( CCCoreLib
		PUBLIC
			CC_CORE_LIB_USES_DOUBLE
	)
else()
	target_compile_definitions( CCCoreLib
		PUBLIC
			CC_CORE_LIB_USES_FLOAT
	)
endif()

# Windows-specific flags
if ( WIN32 )
	# VLD for mem leak checking
	option( CCCORELIB_USE_VISUAL_LEAK_DETECTOR
		"Check to activate compilation (in debug) with Visual Leak Detector"
		OFF
	)

	if ( CCCORELIB_USE_VISUAL_LEAK_DETECTOR )
		target_compile_definitions( CCCoreLib
			PRIVATE
				CC_CORE_LIB_USES_VLD
		)
	endif()

	# Disable SECURE_SCL
	# See https://channel9.msdn.com/shows/Going+Deep/STL-Iterator-Debugging-and-Secure-SCL/
	target_compile_definitions( CCCoreLib
		PRIVATE
			"$<$<CONFIG:RELEASE>:_SECURE_SCL=0>"
	)

	target_compile_definitions( CCCoreLib
		PRIVATE
			_CRT_SECURE_NO_WARNINGS
			__STDC_LIMIT_MACROS
			NOMINMAX
	)
endif()

# TBB (optional)
# Must come before CGAL so it can use TBB properly
if ( CCCORELIB_USE_TBB )
	find_package( TBB COMPONENTS tbb CONFIG )

	if ( TBB_FOUND )
		if ( ${TBB_VERSION} VERSION_GREATER 2021.0.0 )
			target_link_libraries( CCCoreLib
				PUBLIC
				TBB::tbb
			)
		else()
			target_link_libraries( CCCoreLib
				PUBLIC
				${TBB_IMPORTED_TARGETS}
			)
		endif()
		target_compile_definitions( CCCoreLib
			PUBLIC
				CC_CORE_LIB_USES_TBB
				TBB_VERSION="${TBB_VERSION}"
		)
	endif()
endif()

# CGAL (optional)
if ( CCCORELIB_USE_CGAL )
	find_package( CGAL REQUIRED )

	if( WIN32 )
		# Need to force the visibility of these variables so that we can use them later
		set (GMP_LIBRARIES ${GMP_LIBRARIES} PARENT_SCOPE) 
		set (MPFR_LIBRARIES ${MPFR_LIBRARIES} PARENT_SCOPE) 
	endif()

	if ( CCCORELIB_USE_TBB )
		if ( TBB_FOUND )
			# Once Linux libcgal-dev >= 5.0, target_compile_definitions replaced by:
			#  CGAL_target_use_TBB( CCCoreLib )

			target_compile_definitions( CCCoreLib
				PRIVATE
					CGAL_LINKED_WITH_TBB
					NOMINMAX
			)
		else()
			message( WARNING "CGAL cannot compile with TBB (TBB not found)" )
		endif()
	endif()

	target_link_libraries( CCCoreLib
		PUBLIC
			CGAL::CGAL
	)

	target_compile_definitions( CCCoreLib
		PUBLIC
			CC_CORE_LIB_USES_CGAL_LIB
	)
endif()

# QT (optional)
if ( CCCORELIB_USE_QT_CONCURRENT )
	find_package( Qt5
		COMPONENTS
			Concurrent
		REQUIRED
	)

    set_target_properties( CCCoreLib PROPERTIES
        AUTOMOC OFF
        AUTORCC OFF
        AUTOUIC OFF
    )

	target_link_libraries( CCCoreLib
		PUBLIC
            Qt5::Concurrent
	)

	target_compile_definitions( CCCoreLib
		PUBLIC
			CC_CORE_LIB_USES_QT_CONCURRENT
	)
endif()

# Install
# See: https://cliutils.gitlab.io/modern-cmake/chapters/install/installing.html
install(
	TARGETS
		CCCoreLib
	EXPORT
		CCCoreLib-targets
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	INCLUDES DESTINATION include
)

install(
	EXPORT
		CCCoreLib-targets
	FILE
		CCCoreLibTargets.cmake
	NAMESPACE
		CCCoreLib::
	DESTINATION
		lib/cmake/CCCoreLib
)

# CMake Package Files
include( CMakePackageConfigHelpers )

configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

write_basic_package_version_file( CCCoreLibConfigVersion.cmake
	VERSION
		${PACKAGE_VERSION}
	COMPATIBILITY
		AnyNewerVersion
)

install(
	FILES
		"${CMAKE_CURRENT_BINARY_DIR}/CCCoreLibConfigVersion.cmake"
		"${CMAKE_CURRENT_BINARY_DIR}/CCCoreLibConfig.cmake"
	DESTINATION
		lib/cmake/CCCoreLib
)

# Export
export(
	TARGETS
		CCCoreLib
	NAMESPACE
		CCCoreLib::
	FILE
		CCCoreLibTargets.cmake
)
