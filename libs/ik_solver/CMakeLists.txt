# Create a library called "myLib1" (in Linux, this library is created
# with the name of either libmyLib1.a or myLib1.so).
add_library (ik_solver)

# Indicate what directories should be added to the include file search
# path when using this library.
target_include_directories(ik_solver PUBLIC
  # list of directories:
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

target_sources(ik_solver PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/robot.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/DHParameters.cpp
  )
