# Declare dependencies
macro (setup_testcase_dependencies)
  link_hifi_libraries(shared test-utils entities entities-renderer)
  package_libraries_for_deployment()
endmacro ()

setup_hifi_testcase()
