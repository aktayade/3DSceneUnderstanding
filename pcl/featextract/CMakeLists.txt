CMAKE_MINIMUM_REQUIRED(VERSION 2.8 FATAL_ERROR)

########### Project name ###########
SET(PACKAGE featextract)
PROJECT(${PACKAGE})

########### Find packages ###########
FIND_PACKAGE(PCL 1.5 REQUIRED)

#IF(UNIX AND NOT WIN32)
# SET(CMAKE_CXX_FLAGS "-L.")
# SET(CMAKE_EXE_LINKER_FLAGS "-L.")
#ENDIF()

INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})

########### Subdirectories ###############
ADD_SUBDIRECTORY(src)

