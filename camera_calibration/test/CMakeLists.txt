find_package(ament_cmake_pytest REQUIRED)
find_package(ament_cmake_flake8 REQUIRED)
find_package(ament_cmake_pep257 REQUIRED)
find_package(ament_cmake_copyright REQUIRED)

ament_add_pytest_test(test_directed.py "test_directed.py")
ament_add_pytest_test(test_multiple_boards.py "test_multiple_boards.py")

ament_copyright()
ament_pep257()
ament_flake8()
