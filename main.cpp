/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#include <QApplication>

#include <cg3/viewer/mainwindow.h>

#include <cg3/viewer/managers/booleans_manager.h>
#include <cg3/viewer/managers/eigenmesh_manager.h>
#include <GUI/managers/fouraxisfabricationmanager.h>

using namespace cg3;

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    cg3::viewer::MainWindow gui;

    //Add four-axis-fabrication manager
    FourAxisFabricationManager fm(&gui);
    const int fmId = gui.addManager(&fm, "Four Axis Fabrication Manager");
    CG3_SUPPRESS_WARNING(fmId);

    //Add boolean manager manager
    cg3::viewer::BooleansManager bm(&gui);
    const int bmId = gui.addManager(&bm, "Booleans Manager");
    CG3_SUPPRESS_WARNING(bmId);

    //Add eigen mesh manager
    cg3::viewer::EigenMeshManager em(&gui);
    const int emId = gui.addManager(&em, "EigenMesh Manager");
    CG3_SUPPRESS_WARNING(emId);

    //Open four-axis fabrication manager as default manager
    gui.setCurrentIndexToolBox(fmId);
    gui.update();
    gui.show();

    return app.exec();
}
