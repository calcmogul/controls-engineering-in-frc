cmake_minimum_required(VERSION 3.21)

project(DoubleIntegratorMinimumTime)

file(GLOB src ./*.cpp)
get_filename_component(exec ${src} NAME_WE)

include(FetchContent)

fetchcontent_declare(
    Sleipnir
    GIT_REPOSITORY https://github.com/SleipnirGroup/Sleipnir.git
    # main on 2024-02-23
    GIT_TAG e4fdf64633b62416e07da8c10ead4acba46673c4
)
fetchcontent_makeavailable(Sleipnir)

fetchcontent_declare(
    fmt
    GIT_REPOSITORY https://github.com/fmtlib/fmt.git
    GIT_TAG 9.1.0
)
fetchcontent_makeavailable(fmt)

add_executable(${exec} ${src})
target_link_libraries(${exec} PRIVATE Sleipnir fmt)
