CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
project(Dogbot_Project C CXX)

find_package(Aria REQUIRED)
find_package(BaseArnl REQUIRED)
find_package(Arnl)
find_package(SonArnl)


include_directories(
    .
    ${Aria_INCLUDE_DIRS}
    ${SonArnl_INCLUDE_DIRS}
	${Arnl_INCLUDE_DIRS}
    ${robotServer_INCLUDE_DIRS}
    ${behaviourLibrary_INCLUDE_DIRS}
)

set( LIBRARIES
    ${LIBRARIES}
    ${Aria_LIBRARIES}
    ${robotServer_LIBRARIES}
${SonArnl_LIBRARIES}
${Arnl_LIBRARIES}
    ${behaviourLibrary_LIBRARIES}
)

file(GLOB CURRENT_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
file(GLOB CURRENT_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.c*")

add_executable(${PROJECT_NAME} ${CURRENT_SOURCES} ${CURRENT_HEADERS})

target_link_libraries(${PROJECT_NAME} ${LIBRARIES})
