# Project's name
cmake_minimum_required(VERSION 3.1)
PROJECT ( LunaBotics C CXX )



# Specify the minimum version for CMake



#add_definitions(-std=c++11)
add_definitions(-std=c++14 -O3)
# Set the output folder where your program will be created
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(PHOENIX_LIBRARY_PATH /usr/include/ctre/lib)
set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package (Threads REQUIRED)
find_package(SDL2 REQUIRED)

# The following folder will be included
set(CMAKE_CXX_STANDARD 17)
add_executable(example ${PROJECT_SOURCE_DIR}/example.cpp)


# Determine processor arch for compiling
if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf") # Jetson TK1 / Pi
	set(PROCESSOR_ARCH raspberry)
elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu") # Jetson TX2
	set(PROCESSOR_ARCH jetsontx)
else()
 	set(PROCESSOR_ARCH x86-64)
endif()


if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf") # Jetson TK1 / Pi
	set(PHOENIX_LIBS_DIR ${PHOENIX_LIBRARY_PATH}/raspberry)
	add_executable(testbed ${PROJECT_SOURCE_DIR}/testbed.cpp)
	target_link_libraries(testbed ${SDL2_LIBRARIES})
	target_link_libraries(testbed Threads::Threads)
    target_link_libraries(testbed ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixCCI.so)
	target_link_libraries(testbed ${PHOENIX_LIBS_DIR}/libCTRE_Phoenix.so)
	target_link_libraries(testbed wiringPi)

elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu") # Jetson TX2
	option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)
	if (NOT LINK_SHARED_ZED AND MSVC)
		message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
	endif()

	if(COMMAND cmake_policy)
		cmake_policy(SET CMP0003 OLD)
		cmake_policy(SET CMP0015 OLD)
	endif(COMMAND cmake_policy)
	if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
	SET(CMAKE_BUILD_TYPE "RelWithDebInfo")
	endif()
	
	add_executable(autonomous ${PROJECT_SOURCE_DIR}/jettsonCode/main.cpp)
	find_package(ZED 3 REQUIRED)
	find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)
	
	include_directories(${CUDA_INCLUDE_DIRS})
	include_directories(${ZED_INCLUDE_DIRS})

	link_directories(${ZED_LIBRARY_DIR})
	link_directories(${CUDA_LIBRARY_DIRS})
	
	set(PHOENIX_LIBS_DIR ${PHOENIX_LIBRARY_PATH}/jetsontx)

    target_link_libraries(autonomous ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixCCI.so)
	target_link_libraries(autonomous ${PHOENIX_LIBS_DIR}/libCTRE_Phoenix.so)

	target_include_directories(autonomous PUBLIC ${CMAKE_SOURCE_DIR})
	target_link_libraries(autonomous pthread)
    if (LINK_SHARED_ZED)
		SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
	else()
		SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
	endif()

	TARGET_LINK_LIBRARIES(zedshit ${ZED_LIBS})

else()
	set(PHOENIX_LIBS_DIR ${PHOENIX_LIBRARY_PATH}/x86-64)
endif()



target_link_libraries(example ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixCCI.so)
target_link_libraries(example ${PHOENIX_LIBS_DIR}/libCTRE_Phoenix.so)
target_link_libraries(example Threads::Threads)
target_link_libraries(example ${SDL2_LIBRARIES})
target_link_libraries(example stdc++fs)
target_link_libraries(example pthread)

add_custom_command(TARGET example POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${PHOENIX_LIBRARY_PATH}
        $<TARGET_FILE_DIR:example>)


