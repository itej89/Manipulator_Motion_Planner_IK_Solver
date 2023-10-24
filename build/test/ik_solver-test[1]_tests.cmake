add_test( IK_TRAJECTORY_TEST.this_should_pass /home/tej/Documents/Maryland/Semester4/808x/HW/OCt31/Manipulator_Motion_Planner_IK_Solver/build/test/ik_solver-test [==[--gtest_filter=IK_TRAJECTORY_TEST.this_should_pass]==] --gtest_also_run_disabled_tests)
set_tests_properties( IK_TRAJECTORY_TEST.this_should_pass PROPERTIES WORKING_DIRECTORY /home/tej/Documents/Maryland/Semester4/808x/HW/OCt31/Manipulator_Motion_Planner_IK_Solver/build/test SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==])
set( ik_solver-test_TESTS IK_TRAJECTORY_TEST.this_should_pass)
