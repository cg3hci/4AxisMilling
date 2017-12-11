/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#include "cg3/viewer/mainwindow.h"
#include "cg3/viewer/managers/dcel_manager/dcel_manager.h"
#include "cg3/viewer/managers/window_manager/window_manager.h"
#include "GUI/managers/fouraxischeckermanager.h"
#include <QApplication>

#include "cg3/viewer/managers/eigenmesh_manager/eigenmesh_manager.h"
#include "cg3/viewer/managers/booleans_manager/booleans_manager.h"
#include <cg3/utilities/string.h>
#include <cg3/utilities/system.h>

#include <typeinfo>       // operator typeid

using namespace cg3;

int main(int argc, char *argv[]) {

    //
    Pointd p1(-35.162235633025972, 31.477472305941479, -0.6277238650953828);
    Pointd p2(-35.089329955168346, 31.464101470722817, -1.9002630311736262);
    Pointd p3(-35.010631205738115, 32.828797361398827, -2.0303997002748098);
    Pointd c1 = (p1 +p2 +p3)/3;
    Vec3 normal((p2-p1).cross(p3-p1));
    normal.normalize();
    normal *= 100;
    SimpleEigenMesh m;
    p1*=200;
    p2*=200;
    p3*=200;
    Pointd c2 = (p1 +p2 +p3)/3;
    p1 -= c2-c1;
    p2 -= c2-c1;
    p3 -= c2-c1;

    m.addVertex(p1);
    m.addVertex(p2);
    m.addVertex(p3);

    m.addVertex(p1 + normal);
    m.addVertex(p2 + normal);
    m.addVertex(p3 + normal);
    m.addFace(0,2,1);
    m.addFace(3,4,5);
    m.addFace(5,2,0);
    m.addFace(5,0,3);
    m.addFace(2,5,1);
    m.addFace(1,5,4);
    m.addFace(1,4,0);
    m.addFace(0,4,3);
    m.saveOnObj("mesh.obj");
    //

    QApplication app(argc, argv);

    MainWindow gui;  // finestra principale, contiene la canvas di QGLViewer

    // Creo un window manager e lo aggiungo alla mainwindow
    WindowManager wm(&gui);
    gui.addManager(&wm, "Window");

    // Creo un dcel manager e lo aggiungo alla mainwindow
    DcelManager d(&gui);
    gui.addManager(&d, "Dcel");

    FourAxisMillingManager fm(&gui);
    const int FAM_M_ID = gui.addManager(&fm, "Four Axis Milling Manager");

    BooleansManager bm(&gui);
    gui.addManager(&bm, "Booleans Manager");

    EigenMeshManager em(&gui);
    gui.addManager(&em, "EigenMesh Manager");

    gui.setCurrentIndexToolBox(FAM_M_ID); // il dcel manager sarà quello visualizzato di default
    gui.updateGlCanvas();
    gui.show();

    return app.exec();
}
