
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src/core)

# To use all *.cpp and *.hpp
FILE(GLOB ALL_HEADER_FILES *.hpp core/*.hpp)
FILE(GLOB ALL_CPP_FILES *.cpp core/*.cpp)

SET(SRCS
	${ALL_HEADER_FILES}
	${ALL_CPP_FILES}
)

SET(extLinkerOption ${PCL_LIBRARIES})
#SET(extLinkerOption ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_<>_LIBRARIES})

ADD_EXECUTABLE(featextract ${SRCS})
TARGET_LINK_LIBRARIES(featextract ${extLinkerOption})

