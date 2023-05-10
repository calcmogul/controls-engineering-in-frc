cmake_minimum_required(VERSION 3.21)

project(DoubleIntegratorMinimumTime)

file(GLOB src ./*.cpp)
get_filename_component(exec ${src} NAME_WE)

include(FetchContent)

FetchContent_Declare(
  Sleipnir
  GIT_REPOSITORY https://github.com/SleipnirGroup/Sleipnir.git
  GIT_TAG        561932a764adfd7a2478b4d7d83e9b2a809a2aa1
)
FetchContent_MakeAvailable(Sleipnir)

FetchContent_Declare(
  fmt
  GIT_REPOSITORY https://github.com/fmtlib/fmt.git
  GIT_TAG        9.1.0
)
FetchContent_MakeAvailable(fmt)

add_executable(${exec} ${src})
target_link_libraries(${exec} PRIVATE Sleipnir fmt)
