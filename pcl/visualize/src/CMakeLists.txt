# To use all *.cpp and *.hpp
FILE(GLOB ALL_HEADER_FILES *.hpp)
FILE(GLOB CPP_FILES *.cpp)

SET(SRCS
	${CPP_FILES}
	${ALL_HEADER_FILES}
)

SET(extLinkerOption ${PCL_LIBRARIES})
#SET(extLinkerOption ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

ADD_EXECUTABLE(pcdbin2pcdascii ${SRCS})
TARGET_LINK_LIBRARIES(pcdbin2pcdascii ${extLinkerOption})

