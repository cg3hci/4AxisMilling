/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#include <QApplication>

#include <cg3/viewer/mainwindow.h>

#include <cg3/viewer/managers/booleans_manager.h>
#include <cg3/viewer/managers/eigenmesh_manager.h>
#include "GUI/managers/fafmanager.h"
#include "GUI/managers/fafsegmentationmanager.h"

using namespace cg3;

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    cg3::viewer::MainWindow gui;

    //Add four-axis-fabrication manager
    FAFManager fm(&gui);
    const int fmId = gui.addManager(&fm, "Four Axis Fabrication Manager");
    CG3_SUPPRESS_WARNING(fmId);

    //Add four-axis-fabrication manager
    FAFSegmentationManager fsm(&gui);
    const int fsmId = gui.addManager(&fsm, "Segmentation Manager");
    CG3_SUPPRESS_WARNING(fsmId);

    //Open four-axis fabrication manager as default manager
    gui.setCurrentManager(fmId);
    gui.update();
    gui.show();

    return app.exec();
}
