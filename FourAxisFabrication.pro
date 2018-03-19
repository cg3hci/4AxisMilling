CONFIG(debug, debug|release){
    DEFINES += DEBUG
}
CONFIG(release, debug|release){
    DEFINES -= DEBUG
    #just uncomment next lines if you want to ignore asserts and got a more optimized binary
    CONFIG += FINAL_RELEASE
}

#CONFIG += c++14

CONFIG += ALL
#CONFIG += SERVER_MODE
#CONFIG += CONVERTER_MODE

ALL {
    #CONFIG += USE_LIBIGL_EIGEN
    CONFIG += MULTI_LABEL_OPTIMIZATION
    CONFIG += CG3_ALL
    include(cg3lib/cg3.pri)
}

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

exists($$(GUROBI_HOME)){
    message (Gurobi)
    INCLUDEPATH += $$(GUROBI_HOME)/include
    LIBS += -L$$(GUROBI_HOME)/lib -lgurobi_g++5.2 -lgurobi75
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

HEADERS += \
    methods/fouraxisfabrication.h \
    methods/faf/faf_data.h \
    methods/faf/faf_utilities.h \
    methods/faf/faf_visibilitycheck.h \
    methods/faf/faf_minimization.h \
    GUI/managers/fouraxisfabricationmanager.h \
    methods/faf/faf_optimization.h \
    methods/faf/faf_smoothing.h \
    methods/faf/faf_boolean.h \
    methods/faf/faf_extract.h

SOURCES += \
    main.cpp \
    methods/fouraxisfabrication.cpp \
    methods/faf/faf_data.cpp \
    methods/faf/faf_utilities.cpp \
    methods/faf/faf_visibilitycheck.cpp \
    methods/faf/faf_minimization.cpp \
    GUI/managers/fouraxisfabricationmanager.cpp \
    methods/faf/faf_optimization.cpp \
    methods/faf/faf_smoothing.cpp \
    methods/faf/faf_boolean.cpp \
    methods/faf/faf_extract.cpp

FORMS += \
    GUI/managers/fouraxisfabricationmanager.ui

DISTFILES += \
    README.txt
