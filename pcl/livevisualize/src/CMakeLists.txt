
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src/core)

# To use all *.cpp and *.hpp
FILE(GLOB ALL_HEADER_FILES core/*.hpp)
FILE(GLOB CPP_FILES *.cpp core/*.cpp)

SET(SRCS
	${CPP_FILES}
	${ALL_HEADER_FILES}
)

SET(extLinkerOption ${PCL_LIBRARIES})
#SET(extLinkerOption ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

ADD_EXECUTABLE(livevisualize ${SRCS})
TARGET_LINK_LIBRARIES(livevisualize ${extLinkerOption})

