

SET(PROJECT_NAME icp)
file(GLOB SOURCE icp.cpp)

find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})


add_library(${PROJECT_NAME} ${SOURCE})
install(TARGETS ${PROJECT_NAME} DESTINATION ../lib)
file(GLOB HEADERS nanoflann.hpp icp.h)
install(FILES ${HEADERS} DESTINATION ../include)


find_package(PCL)
if(PCL_FOUND)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${PROJECT_SOURCE_DIR}/lib)
add_executable(icpExample main.cpp display.cpp timer.cpp )
target_link_libraries(icpExample ${PCL_LIBRARIES} icp)
endif(PCL_FOUND)
