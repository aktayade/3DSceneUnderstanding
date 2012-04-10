CMAKE_MINIMUM_REQUIRED(VERSION 2.8 FATAL_ERROR)

########### Project name ###########
SET(PACKAGE spinextract)
PROJECT(${PACKAGE})

########### Main global variables ###########
#IF(NOT CMAKE_BUILD_TYPE)
# SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel." FORCE)
#ENDIF()

#IF(UNIX AND NOT WIN32)
# SET(CMAKE_CXX_FLAGS "-Wall -Wextra -Wno-unused-parameter")		# Enable lots of warning flags
#ELSE()
# SET(CMAKE_CXX_FLAGS "-Wall -Wsign-promo -fexceptions -fident -mthreads")	# Refer to Stellarium. Flags used on purpose
# SET(CMAKE_EXE_LINKER_FLAGS "-Wsign-promo -fexceptions -fident -mthreads -mwindows --large-address-aware")
#ENDIF()

########### Find packages ###########
#SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake/Modules/)
#FIND_PACKAGE(PCL 1.5 REQUIRED COMPONENTS common io kdtree)
FIND_PACKAGE(PCL 1.5 REQUIRED)

INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})

########### Subdirectories ###############
ADD_SUBDIRECTORY(src) 

