set(hdrs Mapper_MRPT.h MapBuilder_MRPT.h MapBuilder.h MobileRobotSVC_impl.h
    PARENT_SCOPE
    MobileRobotSVC_impl.h
    )

install(FILES ${hdrs} DESTINATION ${INC_INSTALL_DIR}/${PROJECT_NAME_LOWER}
    COMPONENT library)

