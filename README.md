# Vision-Local

if(QT_VERSION_MAJOR EQUAL 6)
    qt6_wrap_cpp(MOC_SOURCES ${HEADERS})
    qt6_wrap_ui(UIC_SOURCES ${UI_FILES})
    qt_add_resources(RESOURCE_SOURCES ${RESOURCE_FILES})
elseif(QT_VERSION_MAJOR EQUAL 5)
    qt5_wrap_cpp(MOC_SOURCES ${HEADERS})
    qt5_wrap_ui(UIC_SOURCES ${UI_FILES})
    qt5_add_resources(RESOURCE_SOURCES ${RESOURCE_FILES})
else()
    qt_wrap_cpp(MOC_SOURCES ${HEADERS})  # Qt4의 경우
    qt_wrap_ui(UIC_SOURCES ${UI_FILES})  # Qt4의 경우
    qt4_add_resources(RESOURCE_SOURCES ${RESOURCE_FILES})  # Qt4의 경우 (수동으로 정의 필요)
endif()
