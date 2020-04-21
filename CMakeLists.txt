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
option( CC-CORE-LIB_USE_CGAL
	"Compile CCCoreLib with CGAL (to enable Delaunay 2.5D triangulation with a GPL-compliant licence)"
	OFF
)
option( CC-CORE-LIB_USE_TBB
	"Compile CCCoreLib with Intel Threading Building Blocks lib (enables some parallel processing )"
	OFF
)
option( CC-CORE-LIB_USE_QT
	"Compile CCCoreLib with Qt (to enable parallel processing)"
	ON
)
option( CC-CORE-LIB_SHARED
	"Compile CCCoreLib as a shared library"
	ON
)
option( CC-CORE-LIB_SCALAR_DOUBLE
	"Define ScalarType as double (instead of float)"
	OFF
)

# Add the library (shared or static)
if ( CC-CORE-LIB_SHARED )
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

if ( CC-CORE-LIB_SCALAR_DOUBLE )
	target_compile_definitions( CCCoreLib
		PUBLIC
			SCALAR_TYPE_DOUBLE
	)
else()
	target_compile_definitions( CCCoreLib
		PUBLIC
			SCALAR_TYPE_FLOAT
	)
endif()

# Windows-specific flags
if ( WIN32 )
	# VLD for mem leak checking
	option( CC-CORE-LIB_USE_VISUAL_LEAK_DETECTOR
		"Check to activate compilation (in debug) with Visual Leak Detector"
		OFF
	)

	if ( CC-CORE-LIB_USE_VISUAL_LEAK_DETECTOR )
		target_compile_definitions( CCCoreLib
			PRIVATE
				USE_VLD
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
if ( CC-CORE-LIB_USE_TBB )
	find_package( TBB COMPONENTS tbb )

	if ( TBB_FOUND )
		target_link_libraries( CCCoreLib
			PUBLIC
				${TBB_IMPORTED_TARGETS}
		)
	
		target_compile_definitions( CCCoreLib
			PUBLIC
				USE_TBB
		)
	endif()
endif()

# CGAL (optional)
if ( CC-CORE-LIB_USE_CGAL )
	find_package( CGAL REQUIRED )

	if ( CC-CORE-LIB_USE_TBB )
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
			USE_CGAL_LIB
	)
endif()

# QT (optional)
if ( CC-CORE-LIB_USE_QT )
	find_package( Qt5
		COMPONENTS
			Concurrent
			Core
			Widgets
		REQUIRED
	)

	set( CMAKE_AUTOMOC ON )
	set( CMAKE_AUTORCC ON )
	set( CMAKE_AUTOUIC ON )

	target_link_libraries( CCCoreLib
		PUBLIC
			Qt5::Concurrent
			Qt5::Widgets
	)

	target_compile_definitions( CCCoreLib
		PUBLIC
			USE_QT
		PRIVATE
			QT_USE_QSTRINGBUILDER
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
		${CMAKE_INSTALL_PREFIX}/cmake/CCCoreLib
)

# CMake Package Files
include( CMakePackageConfigHelpers )

write_basic_package_version_file( CCCoreLibConfigVersion.cmake
	VERSION
		${PACKAGE_VERSION}
	COMPATIBILITY
		AnyNewerVersion
)

install(
	FILES
		"${CMAKE_CURRENT_BINARY_DIR}/CCCoreLibConfigVersion.cmake"
	DESTINATION
		${CMAKE_INSTALL_PREFIX}/cmake/CCCoreLib
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
