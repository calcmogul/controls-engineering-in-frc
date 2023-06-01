cmake_minimum_required(VERSION 3.21)

project(DoubleIntegratorMinimumTime)

file(GLOB src ./*.cpp)
get_filename_component(exec ${src} NAME_WE)

include(FetchContent)

FetchContent_Declare(
  Sleipnir
  GIT_REPOSITORY https://github.com/SleipnirGroup/Sleipnir.git
  GIT_TAG        0d20a9f712f1e997f734038a977392d8bc6a19f4
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
