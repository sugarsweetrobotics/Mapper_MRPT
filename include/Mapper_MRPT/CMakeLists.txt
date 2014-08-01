set(hdrs Mapper_MRPT.h MapBuilder.h MapBuilder_MRPT.h
    PARENT_SCOPE
    )

install(FILES ${hdrs} DESTINATION ${INC_INSTALL_DIR}/${PROJECT_NAME_LOWER}
    COMPONENT library)

