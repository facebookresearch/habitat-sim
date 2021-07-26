find_package(Corrade REQUIRED Utility)

corrade_add_test(coreLoggingTest LoggingTest.cpp LIBRARIES core Corrade::Utility)
set_tests_properties(coreLoggingTest PROPERTIES ENVIRONMENT HABITAT_SIM_LOG="")
