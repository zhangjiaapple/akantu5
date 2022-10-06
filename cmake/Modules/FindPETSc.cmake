#===============================================================================
# @file   FindPETSc.cmake
#
# @author Aurelia Isabel Cuba Ramos <aurelia.cubaramos@epfl.ch>
# @author Nicolas Richart <nicolas.richart@epfl.ch>
#
# @date creation: Sun Oct 19 2014
# @date last modification: Wed Jun 10 2020
#
# @brief  FindPackage for the PETSc library
#
#
# @section LICENSE
#
# Copyright (©) 2015-2021 EPFL (Ecole Polytechnique Fédérale de Lausanne)
# Laboratory (LSMS - Laboratoire de Simulation en Mécanique des Solides)
#
# Akantu is free software: you can redistribute it and/or modify it under the
# terms of the GNU Lesser General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option) any
# later version.
# 
# Akantu is distributed in the hope that it will be useful, but WITHOUT ANY
# WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
# A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more
# details.
# 
# You should have received a copy of the GNU Lesser General Public License along
# with Akantu. If not, see <http://www.gnu.org/licenses/>.
#
#===============================================================================


# - Try to find PETSc
#  PETSC_FOUND         - system has PETSc
#  PETSC_INCLUDE_DIRS  - the PETSc include directories
#  PETSC_LIBRARIES     - Link these to use PETSc
#  PETSC_VERSION       - Version string (MAJOR.MINOR.SUBMINOR)

if(PETSc_FIND_REQUIRED)
  find_package(PkgConfig REQUIRED)
else()
  find_package(PkgConfig QUIET)
  if(NOT PKG_CONFIG_FOUND)
    return()
  endif()
endif()

pkg_search_module(_petsc PETSc)

# Some debug code
#get_property(_vars DIRECTORY PROPERTY VARIABLES)
#foreach(_var ${_vars})
#  if ("${_var}" MATCHES "^_petsc")
#    message("${_var} -> ${${_var}}")
#  endif()
#endforeach()

if(_petsc_FOUND AND _petsc_VERSION)
  set(PETSC_VERSION ${_petsc_VERSION})
endif()

if(_petsc_FOUND AND (NOT PETSC_LIBRARIES))
  set(_petsc_libs)
  foreach(_lib ${_petsc_LIBRARIES})
    string(TOUPPER "${_lib}" _u_lib)
    find_library(PETSC_LIBRARY_${_u_lib} ${_lib} PATHS ${_petsc_LIBRARY_DIRS})
    list(APPEND _petsc_libs ${PETSC_LIBRARY_${_u_lib}})
    mark_as_advanced(PETSC_LIBRARY_${_u_lib})
  endforeach()

  set(PETSC_LIBRARIES ${_petsc_libs} CACHE INTERNAL "")
  set(PETSC_INCLUDE_DIRS ${_petsc_INCLUDE_DIRS} CACHE INTERNAL "")
  if(NOT TARGET petsc::petsc)
    add_library(petsc::petsc INTERFACE IMPORTED)
    set_property(TARGET petsc::petsc PROPERTY INTERFACE_LINK_LIBRARIES ${PETSC_LIBRARIES})
    set_property(TARGET petsc::petsc PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${PETSC_INCLUDE_DIRS})
  endif()
endif()

include (FindPackageHandleStandardArgs)
find_package_handle_standard_args(PETSc
  REQUIRED_VARS PETSC_LIBRARIES PETSC_INCLUDE_DIRS
  VERSION_VAR PETSC_VERSION)
