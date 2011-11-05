# Copyright 2011, Florent Lamiraux CNRS
#
# This file is part of sot-test.
# sot-test is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# sot-test is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# sot-test. If not, see <http://www.gnu.org/licenses/>.

INCLUDE(../cmake/python.cmake)
# The main include dir
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)

# The main library name
SET(LIBRARY_NAME ${PROJECT_NAME})
SET(${LIBRARY_NAME}_SOURCES
  stepper.cc
  stepper.hh
  )
ADD_LIBRARY(${LIBRARY_NAME}
  SHARED
  ${${LIBRARY_NAME}_SOURCES}
  )

SET_TARGET_PROPERTIES(${LIBRARY_NAME}
  PROPERTIES
  SOVERSION ${PROJECT_VERSION}
  INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib
  )

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} sot-dynamic)

INSTALL(TARGETS ${LIBRARY_NAME} 
  DESTINATION lib)

FINDPYTHON()

# Bindings Python
DYNAMIC_GRAPH_PYTHON_MODULE("sot/dynamics/test" ${LIBRARY_NAME} wrap)

INSTALL(FILES stepping.py
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/dynamics/test
  )
