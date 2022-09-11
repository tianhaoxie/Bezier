if(TARGET igl::core)
    return()
endif()

include(FetchContent)
set(FETCHCONTENT_SOURCE_DIR_LIBIGL "/home/xth/research/bezier/bezier_cpp/thirdparty/libigl")
FetchContent_Declare(
    libigl
    GIT_REPOSITORY https://github.com/libigl/libigl.git
    GIT_TAG <TARGET_SHA1>
)
FetchContent_MakeAvailable(libigl)
