set(TARGET_NAME physics)
setup_hifi_library()

link_hifi_libraries(shared fbx entities gpu model)
add_dependency_external_projects(vhacd)

find_package(VHACD REQUIRED)

target_include_directories(${TARGET_NAME} PUBLIC ${VHACD_INCLUDE_DIRS})

target_bullet()
