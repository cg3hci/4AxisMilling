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
    LIBS += -L$$(GUROBI_HOME)/lib -lgurobi_g++5.2 -lgurobi70
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
    GUI/managers/fouraxischeckermanager.h \
    fouraxis/fouraxis.h

SOURCES += \
    main.cpp \
    GUI/managers/fouraxischeckermanager.cpp \
    fouraxis/fouraxis.cpp

FORMS += \
    GUI/managers/fouraxischeckermanager.ui

DISTFILES += \
    README.txt
