# To use all *.cpp and *.hpp
#FILE(GLOB ALL_HEADER_FILES *.hpp)
#FILE(GLOB CPP_FILES *.cpp)

SET(spinextract_SRCS
	${CPP_FILES}
	${ALL_HEADER_FILES}
)

SET(extLinkerOption ${PCL_LIBRARIES})
#SET(extLinkerOption ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

ADD_EXECUTABLE(spinextract main.cpp)#${spinextract_SRCS})
TARGET_LINK_LIBRARIES(spinextract ${extLinkerOption})

