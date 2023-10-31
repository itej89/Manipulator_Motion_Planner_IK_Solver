# Manipulator_Motion_Planner_IK_Solver

Team Members:

- Krishna Hundekari

- Tej Kiran
  
- Abrarudin Syed

# C++ Boilerplate v2 Badges
![CICD Workflow status](https://github.com/KrishnaH96/Manipulator_Motion_Planner_IK_Solver/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg) [![codecov](https://codecov.io/gh/KrishnaH96/Manipulator_Motion_Planner_IK_Solver/branch/main/graph/badge.svg)](https://codecov.io/gh/KrishnaH96/Manipulator_Motion_Planner_IK_Solver)


# [Link for the AIP Log Sheet](https://docs.google.com/spreadsheets/d/13-OB5Zy51qPaeGvC6LTy-5HckD2iuU5O/edit?usp=sharing&ouid=116812388978309632579&rtpof=true&sd=true)


## Proposal Video:

https://github.com/KrishnaH96/Manipulator_Motion_Planner_IK_Solver/assets/113392023/f795cc02-591a-446a-943f-32bf4f3d80d0



## Steps to build the library and run test cases
``` bash
# Configure the project and generate a native build system:
  # Must re-run this command whenever any CMakeLists.txt file has been changed.
  cmake -S ./ -B build/
# Compile and build the project:
  # rebuild only files that are modified since the last build
  cmake --build build/
  # or rebuild everything from scracth
  cmake --build build/ --clean-first
  # to see verbose output, do:
  cmake --build build/ --verbose
# Run tests:
  cd build/; ctest; cd -
# Clean and start over:
  rm -rf build/
```


## Steps to geneate code coverage report
```bash
# if you don't have gcovr or lcov installed, do:
  sudo apt-get install gcovr lcov
# Set the build type to Debug and WANT_COVERAGE=ON
  cmake -D WANT_COVERAGE=ON -D CMAKE_BUILD_TYPE=Debug -S ./ -B build/
# Now, do a clean compile, run unit test, and generate the coverage report
  cmake --build build/ --clean-first --target all test_coverage
# open a web browser to browse the test coverage report
  open build/test_coverage/index.html
## Steps to install eigen library
```bash


# Install eigen library
sudo apt install libeigen3-dev
# Install numpy
pip3 install numpy
```