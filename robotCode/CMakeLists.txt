#set minimum cmake version and c++ standard
cmake_minimum_required(VERSION 3.1)
PROJECT ( LunaBotics C CXX )
add_definitions(-std=c++14 -O3)
set(CMAKE_CXX_STANDARD 17)


# Set the output folder where your program will be created
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

find_package (Threads REQUIRED)
find_package(SDL2 REQUIRED)

set(CTRE_OBJECTS 
	libCTRE_PhoenixCCI.so
	libCTRE_Phoenix.so
	)

#define include directory for CTRE .so's that get manually installed
set(PHOENIX_LIBRARY_PATH /usr/include/ctre/lib)
set(THREADS_PREFER_PTHREAD_FLAG ON)


# Determine processor arch for compiling

if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf") # Jetson TK1 / Pi
	set(PROCESSOR_ARCH raspberry)
	set(TARGET_EXECUTABLES 
		example
		testbed
		)
elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu") # Jetson TX2
	set(PROCESSOR_ARCH jetsontx)
	set(TARGET_EXECUTABLES 
		example
		testbed
		autonomy
		)
else()
 	set(PROCESSOR_ARCH x86-64)
	 set(TARGET_EXECUTABLES 
		example
		testbed
		autonomy
		)
endif()



add_executable(example ${PROJECT_SOURCE_DIR}/example.cpp)

if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf") 
	### For Rasperry Pi build testbed along with defaults
	set(PHOENIX_LIBS_DIR ${PHOENIX_LIBRARY_PATH}/${PROCESSOR_ARCH})
	add_executable(testbed ${PROJECT_SOURCE_DIR}/testbed.cpp)


    target_link_libraries(testbed ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixCCI.so)
	target_link_libraries(testbed ${PHOENIX_LIBS_DIR}/libCTRE_Phoenix.so)

	target_link_libraries(testbed Threads::Threads)
	target_link_libraries(testbed wiringPi)

<<<<<<< HEAD
elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu")
	### FOR JETSON/NX build autonomy target along with defaults
	find_package(ZED 3 REQUIRED)
	find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)

	add_executable(autonomous ${PROJECT_SOURCE_DIR}/jettsonCode/main.cpp)

	include_directories(${CUDA_INCLUDE_DIRS})
	include_directories(${ZED_INCLUDE_DIRS})
=======

elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu") # Jetson TX2
	option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)
	if (NOT LINK_SHARED_ZED AND MSVC)
		message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
	endif()
>>>>>>> c709221fb443f4210c3a5e02e243b4c80d51afc8

	link_directories(${ZED_LIBRARY_DIR})
	link_directories(${CUDA_LIBRARY_DIRS})


	option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)
	if(COMMAND cmake_policy)
		cmake_policy(SET CMP0003 OLD)
		cmake_policy(SET CMP0015 OLD)
	endif(COMMAND cmake_policy)
	if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
	SET(CMAKE_BUILD_TYPE "RelWithDebInfo")
	endif()
	
	set(PHOENIX_LIBS_DIR ${PHOENIX_LIBRARY_PATH}/${PROCESSOR_ARCH})

    target_link_libraries(autonomous ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixCCI.so)
	target_link_libraries(autonomous ${PHOENIX_LIBS_DIR}/libCTRE_Phoenix.so)

	target_include_directories(autonomous PUBLIC ${CMAKE_SOURCE_DIR})

	target_link_libraries(autonomous pthread)

    if (LINK_SHARED_ZED)
		SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
	else()
		SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
	endif()

	TARGET_LINK_LIBRARIES(autonomous ${ZED_LIBS})

else()
	set(PHOENIX_LIBS_DIR ${PHOENIX_LIBRARY_PATH}/x86-64)
endif()



### BUILD EXAMPLE CODE FOR ALL TARGETS
target_link_libraries(example ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixCCI.so)
target_link_libraries(example ${PHOENIX_LIBS_DIR}/libCTRE_Phoenix.so)
target_link_libraries(example ${SDL2_LIBRARIES})
target_link_libraries(example Threads::Threads)
target_link_libraries(example stdc++fs)
target_link_libraries(example pthread)

add_custom_command(TARGET example POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${PHOENIX_LIBRARY_PATH}
        $<TARGET_FILE_DIR:example>)


