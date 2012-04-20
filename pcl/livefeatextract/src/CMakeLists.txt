
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src/core ${CMAKE_SOURCE_DIR}/src/openni)

# To use all *.cpp and *.hpp
FILE(GLOB ALL_HEADER_FILES *.hpp core/*.hpp openni/*.hpp)
FILE(GLOB ALL_CPP_FILES *.cpp core/*.cpp openni/*.cpp)

SET(SRCS
	${ALL_HEADER_FILES}
	${ALL_CPP_FILES}
)

SET(extLinkerOption ${PCL_LIBRARIES})
#SET(extLinkerOption ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_<>_LIBRARIES})

ADD_EXECUTABLE(livefeatextract ${SRCS})
TARGET_LINK_LIBRARIES(livefeatextract ${extLinkerOption})

