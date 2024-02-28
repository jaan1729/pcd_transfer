cmake_minimum_required(VERSION 3.10.2)
set(CMAKE_TOOLCHAIN_FILE "/vcpkg/scripts/buildsystems/vcpkg.cmake")
project(stpcc)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS iostreams serialization program_options system REQUIRED)
find_package(ZLIB REQUIRED)


set(CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)


include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${ZLIB_INCLUDE_DIRS}
    ${GZIP_INCLUDE_DIRS}
    /vcpkg/installed/x64-linux/include/gzip #Temporary solution to get gzip headers manually
)

message("Boost include dirs: ${Boost_INCLUDE_DIRS}")
message("Boost libraries: ${Boost_LIBRARIES}")
message("OpenCV include dirs: ${OpenCV_INCLUDE_DIRS} "
        "OpenCV libraries: ${OpenCV_LIBS}")


add_library(pcc_common src/utils.cpp
            src/encoder.cpp
            src/pcc_module.cpp
            src/decoder.cpp
            src/io.cpp
            src/archive.cpp
            src/serialize.cpp
            )

target_link_libraries(pcc_common PUBLIC
                        ${OpenCV_LIBS}
                        ${Boost_LIBRARIES}
                        ${ZLIB_LIBRARIES}
                        ${GZIP}
                        )

target_include_directories(pcc_common PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)


add_executable(pcc_encoder src/pcc_encoder.cpp)
add_executable(pcc_decoder src/pcc_decoder.cpp)
# add_executable(pcc_file_encoder src/pcc_file_encoder.cpp)
# add_executable(pcc_file_decoder src/pcc_file_decoder.cpp)

add_executable(pcc_serialization src/pcc_serialization.cpp)

# target_include_directories(pcc_encoder PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# target_include_directories(pcc_decoder PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(pcc_encoder ${ZLIB_LIBRARIES} pcc_common)
target_link_libraries(pcc_decoder ${ZLIB_LIBRARIES} pcc_common)
# target_link_libraries(pcc_file_encoder ${ZLIB_LIBRARIES} pcc_common)
# target_link_libraries(pcc_file_decoder ${ZLIB_LIBRARIES} pcc_common)
target_link_libraries(pcc_serialization ${ZLIB_LIBRARIES} pcc_common)