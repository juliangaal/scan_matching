function(include_CATCH2)
    Include(FetchContent)
    FetchContent_Declare(
            Catch2
            GIT_REPOSITORY https://github.com/catchorg/Catch2.git
            GIT_TAG        v3.0.0-preview3
    )
    FetchContent_MakeAvailable(Catch2)
endfunction()

function(include_fmt)
    FetchContent_Declare(
            fmt
            GIT_REPOSITORY https://github.com/fmtlib/fmt.git
            GIT_TAG        8.0.1
    )
    FetchContent_MakeAvailable(fmt)
endfunction()