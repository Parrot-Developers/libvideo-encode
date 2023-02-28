cmake_minimum_required(VERSION 3.15)
project(video-encode VERSION 1.0)

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

# video-encode
set(LIB_SOURCES
        src/venc.c
)

add_library(${PROJECT_NAME} ${LIB_SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "VENC_API_EXPORTS")
target_compile_options(${PROJECT_NAME} PRIVATE "-fvisibility=hidden")
target_compile_options(${PROJECT_NAME} PRIVATE "-std=gnu99")

target_link_libraries(video-encode pomp ulog video-defs video-encode-core)

# video-encode-core
set(LIB_SOURCES
        core/src/venc_core.c
        core/src/venc_h264.c
        core/src/venc_h265.c
)

add_library(${PROJECT_NAME}-core ${LIB_SOURCES})

target_include_directories(${PROJECT_NAME}-core PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core/include>
                            $<INSTALL_INTERFACE:include>
)

target_compile_definitions(${PROJECT_NAME}-core PRIVATE "VENC_API_EXPORTS")
target_compile_options(${PROJECT_NAME}-core PRIVATE "-fvisibility=hidden")
target_compile_options(${PROJECT_NAME}-core PRIVATE "-std=gnu99")

if(WIN32)
  target_link_libraries(${PROJECT_NAME}-core ws2_32)
endif()                 

target_link_libraries(${PROJECT_NAME}-core
                        futils
                        h264
                        h265
                        media-buffers
                        media-buffers-memory
                        media-buffers-memory-generic
                        pomp
                        ulog
                        video-defs
                        video-metadata
                        video-streaming
)

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}-targets
    PUBLIC_HEADER DESTINATION include
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
