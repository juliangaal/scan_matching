include(FetchContent)

FetchContent_Declare(
        toml11
        GIT_REPOSITORY https://github.com/ToruNiina/toml11.git
        GIT_TAG        v3.7.1
)

set(FMT_TEST FF CACHE INTERNAL "disabling fmt tests")

FetchContent_Declare(
        fmt
        GIT_REPOSITORY https://github.com/fmtlib/fmt.git
        GIT_TAG        10.0.0
)

FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG        v3.3.2
)

FetchContent_MakeAvailable(toml11 fmt Catch2)