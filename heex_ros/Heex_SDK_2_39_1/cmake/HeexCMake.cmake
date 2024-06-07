option(WITH_CLANG_TIDY "Use clang-tidy linter" OFF)

# Steps to build with clang-tidy linter
if(WITH_CLANG_TIDY)
    # check for clang-tidy-17 program
    find_program(CLANG_TIDY_EXE NAMES clang-tidy-17)

    if (CLANG_TIDY_EXE)
        message(STATUS "Found clang-tidy: ${CLANG_TIDY_EXE}")
        execute_process(COMMAND ${CLANG_TIDY_EXE} --version)
        set(CLANG_TIDY_FOUND TRUE)
        # set build with clang-tidy for targets
        set(CMAKE_CXX_CLANG_TIDY ${CLANG_TIDY_EXE})
    else()
        message(WARNING "clang-tidy not found.")
    endif()
endif(WITH_CLANG_TIDY)