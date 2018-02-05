/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#include <cg3/viewer/mainwindow.h>
#include <cg3/viewer/managers/dcel_manager.h>
#include <GUI/managers/fouraxischeckermanager.h>
#include <QApplication>

#include <cg3/viewer/managers/eigenmesh_manager.h>
#include <cg3/viewer/managers/booleans_manager.h>
#include <cg3/utilities/string.h>
#include <cg3/utilities/system.h>

#include <typeinfo>       // operator typeid

using namespace cg3;

int main(int argc, char *argv[]) {

    QApplication app(argc, argv);

    cg3::viewer::MainWindow gui;  // finestra principale, contiene la canvas di QGLViewer

    // Creo un dcel manager e lo aggiungo alla mainwindow
    cg3::viewer::DcelManager d(&gui);
    gui.addManager(&d, "Dcel");

    FourAxisMillingManager fm(&gui);
    const int FAM_M_ID = gui.addManager(&fm, "Four Axis Milling Manager");

    cg3::viewer::BooleansManager bm(&gui);
    gui.addManager(&bm, "Booleans Manager");

    cg3::viewer::EigenMeshManager em(&gui);
    gui.addManager(&em, "EigenMesh Manager");

    gui.setCurrentIndexToolBox(FAM_M_ID); // il dcel manager sarà quello visualizzato di default
    gui.updateGlCanvas();
    gui.show();

    return app.exec();
}
