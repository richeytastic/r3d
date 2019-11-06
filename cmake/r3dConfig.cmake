# =============================================================================
# The r3d CMake configuration file.
#
#           ** File generated automatically, DO NOT MODIFY! ***

# To use from an external project, in your project's CMakeLists.txt add:
#   FIND_PACKAGE( r3d REQUIRED)
#   INCLUDE_DIRECTORIES( r3d ${r3d_INCLUDE_DIRS})
#   LINK_DIRECTORIES( ${r3d_LIBRARY_DIR})
#   TARGET_LINK_LIBRARIES( MY_TARGET_NAME ${r3d_LIBRARIES})
#
# This module defines the following variables:
#   - r3d_FOUND         : True if r3d is found.
#   - r3d_ROOT_DIR      : The root directory where r3d is installed.
#   - r3d_INCLUDE_DIRS  : The r3d include directories.
#   - r3d_LIBRARY_DIR   : The r3d library directory.
#   - r3d_LIBRARIES     : The r3d imported libraries to link to.
#
# =============================================================================

get_filename_component( r3d_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component( r3d_ROOT_DIR  "${r3d_CMAKE_DIR}"           PATH)

set( r3d_INCLUDE_DIRS "${r3d_ROOT_DIR}/../include" CACHE PATH "The r3d include directories.")
set( r3d_LIBRARY_DIR  "${r3d_ROOT_DIR}"            CACHE PATH "The r3d library directory.")

include( "${CMAKE_CURRENT_LIST_DIR}/Macros.cmake")
get_library_suffix( _lsuff)
set( _hints r3d${_lsuff} libr3d${_lsuff})
find_library( r3d_LIBRARIES NAMES ${_hints} PATHS "${r3d_LIBRARY_DIR}/static" "${r3d_LIBRARY_DIR}")
set( r3d_LIBRARIES     ${r3d_LIBRARIES}         CACHE FILEPATH "The r3d imported libraries to link to.")

# handle QUIETLY and REQUIRED args and set r3d_FOUND to TRUE if all listed variables are TRUE
include( "${CMAKE_ROOT}/Modules/FindPackageHandleStandardArgs.cmake")
find_package_handle_standard_args( r3d r3d_FOUND r3d_LIBRARIES r3d_INCLUDE_DIRS)
