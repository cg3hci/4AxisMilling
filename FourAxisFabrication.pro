CONFIG(debug, debug|release){
    DEFINES += DEBUG
}
CONFIG(release, debug|release){
    DEFINES -= DEBUG
    #just uncomment next lines if you want to ignore asserts and got a more optimized binary
    CONFIG += FINAL_RELEASE
}

CONFIG += CG3_STATIC
#CONFIG += c++14

CONFIG += ALL
#CONFIG += SERVER_MODE
#CONFIG += CONVERTER_MODE

#VCGLIB_PATH = /usr/include/vcglib
#LIBIGL_PATH = /usr/include/libigl

CONFIG += MULTI_LABEL_OPTIMIZATION
CONFIG += CLIPPER
CONFIG += CG3_CORE CG3_DATA_STRUCTURES CG3_MESHES CG3_ALGORITHMS CG3_CGAL CG3_LIBIGL CG3_VIEWER CG3_VCGLIB
include(cg3lib/cg3.pri)


message(Included modules: $$MODULES)
FINAL_RELEASE {
    message(Final Release!)
}

FINAL_RELEASE {
    unix:!macx{
        QMAKE_CXXFLAGS_RELEASE -= -g -O2
        QMAKE_CXXFLAGS += -O3 -DNDEBUG
    }
}

!isEmpty(GUROBI_HOME):exists($$(GUROBI_HOME)){
    message (Gurobi)
    INCLUDEPATH += -I $$(GUROBI_HOME)/include
    LIBS += -L$$(GUROBI_HOME)/lib -lgurobi_g++4.2 -lgurobi81
    DEFINES += GUROBI_DEFINED
}


MULTI_LABEL_OPTIMIZATION {
    DEFINES += MULTI_LABEL_OPTIMIZATION_INCLUDED
    INCLUDEPATH += $$PWD/lib/MultiLabelOptimization
    SOURCES += \
        lib/MultiLabelOptimization/GCoptimization.cpp \
        lib/MultiLabelOptimization/graph.cpp \
        lib/MultiLabelOptimization/LinkedBlockList.cpp \
        lib/MultiLabelOptimization/maxflow.cpp \
        lib/MultiLabelOptimization/example.cpp
}

CLIPPER {
    DEFINES += CLIPPER_INCLUDED
    INCLUDEPATH += $$PWD/lib/clipper
    SOURCES += \
        lib/clipper/clipper.cpp
}

HEADERS += \
    methods/fouraxisfabrication.h \
    methods/faf/faf_data.h \
    methods/faf/faf_optimalrotation.h \
    methods/faf/faf_extremes.h \
    methods/faf/faf_visibilitycheck.h \
    methods/faf/faf_frequencies.h \
    methods/faf/faf_extraction.h \
    methods/faf/faf_charts.h \
    methods/faf/faf_association.h \
    methods/faf/includes/view_renderer.h \
    methods/faf/faf_optimization.h \
    methods/faf/faf_smoothing.h \
    methods/faf/faf_various.h \
    GUI/managers/fafmanager.h \
    GUI/managers/fafsegmentationmanager.h \
    methods/faf/faf_split.h

SOURCES += \
    main.cpp \
    methods/faf/faf_data.cpp \
    methods/faf/faf_optimalrotation.cpp \
    methods/faf/faf_extremes.cpp \
    methods/faf/faf_visibilitycheck.cpp \
    methods/faf/faf_frequencies.cpp \
    methods/faf/faf_extraction.cpp \
    methods/faf/faf_charts.cpp \
    methods/faf/faf_association.cpp \
    methods/faf/includes/view_renderer.cpp \
    methods/faf/faf_optimization.cpp \
    methods/faf/faf_smoothing.cpp \
    methods/faf/faf_various.cpp \
    GUI/managers/fafmanager.cpp \
    GUI/managers/fafsegmentationmanager.cpp \
    methods/faf/faf_split.cpp


FORMS += \
    GUI/managers/fafmanager.ui \
    GUI/managers/fafsegmentationmanager.ui

DISTFILES += \
    README.txt

RESOURCES += \
    resources/resources.qrc


