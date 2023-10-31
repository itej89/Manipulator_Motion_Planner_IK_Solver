# Any C++ source files needed to build this target (ik_solver-test).
add_executable(ik_solver-test
  # list of source cpp files:
  test.cpp
  )

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# Any dependent libraires needed to build this target.
target_link_libraries(ik_solver-test PUBLIC
  # list of libraries:
  gtest
  gmock
  ik_solver
  ik_solver Eigen3::Eigen
  )


# Enable CMake’s test runner to discover the tests included in the
# binary, using the GoogleTest CMake module.
gtest_discover_tests(ik_solver-test)
