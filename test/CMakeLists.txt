if (NOT TARGET GTest)
    find_package(GTest CONFIG REQUIRED)
    message("${PROJECT_NAME}: Finding package GTest")
endif ()

# Add the source files
message("${PROJECT_NAME}: Finding tests")
file(GLOB_RECURSE TEST_SOURCES ${PROJECT_SOURCE_DIR}/test/*.cpp)

foreach(TEST_SOURCE ${TEST_SOURCES})
    # Get Test name
    string(REGEX REPLACE "(^.*/|\\.[^.]*$)" "" TEST_NAME ${TEST_SOURCE})

    message("Adding test: ${TEST_NAME}")
    # Add Test
    add_executable(${TEST_NAME} ${TEST_SOURCE})
    target_link_libraries(${TEST_NAME}
            PRIVATE
            ${PROJECT_NAME}
            GTest::gtest
            GTest::gtest_main
            )

    add_test(NAME "${TEST_NAME}GTest" COMMAND ${TEST_NAME})
endforeach()