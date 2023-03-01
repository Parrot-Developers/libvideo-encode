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

target_link_libraries(${PROJECT_NAME} 

    PRIVATE
    futils
    pomp
    ulog
    video-defs
    h264
    h265
    media-buffers
    media-buffers-memory
    media-buffers-memory-generic
    video-defs
    video-metadata
    video-streaming
    video-encode-core
    transport-packet
    )

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
  target_link_libraries(${PROJECT_NAME}-core 
                        # PUBLIC

                        PRIVATE
                        ws2_32
                        futils
                        pomp
                        ulog
                        h264
                        h265
                        media-buffers
                        media-buffers-memory
                        media-buffers-memory-generic
                        video-defs
                        video-metadata
                        video-streaming
                        transport-packet

      )
else()

    target_link_libraries(${PROJECT_NAME}-core
                        # PUBLIC

                        PRIVATE
                        futils
                        pomp
                        ulog
                        h264
                        h265
                        media-buffers
                        media-buffers-memory
                        media-buffers-memory-generic
                        video-defs
                        video-metadata
                        video-streaming
                        transport-packet

    )
endif()


install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}-targets
    PUBLIC_HEADER DESTINATION include
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}-core
    EXPORT ${PROJECT_NAME}-core-targets
    PUBLIC_HEADER DESTINATION include
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
