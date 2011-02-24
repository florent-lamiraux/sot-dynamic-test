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

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/lapack.cmake)
INCLUDE(cmake/cpack.cmake)

SET(PROJECT_NAME sot-dynamic-test)
SET(PROJECT_VERSION 0.1)
SET(PROJECT_DESCRIPTION "Some tests for the stack of tasks on humanoid robots.")
SET(PROJECT_URL "git@github.com:florent-lamiraux/sot-dynamic-test.git")

SET(CUSTOM_HEADER_DIR "sot/dynamic/test")

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR True)

SETUP_PROJECT()

PKG_CONFIG_APPEND_LIBS("sot-dynamic-test")

# Search for dependencies.
# Boost
ADD_REQUIRED_DEPENDENCY("sot-dynamic >= 1.99")

ADD_SUBDIRECTORY(src)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
