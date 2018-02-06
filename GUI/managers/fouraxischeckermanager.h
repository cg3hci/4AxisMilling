#ifndef DRAWMANAGER_H
#define DRAWMANAGER_H

#include <cg3/viewer/interfaces/drawable_object.h>
#include <cg3/viewer/interfaces/drawable_mesh.h>
#include <cg3/viewer/mainwindow.h>
#include <cg3/viewer/utilities/loadersaver.h>

#include <cg3/viewer/pickable_objects/pickable_eigenmesh.h>
#include <cg3/viewer/drawable_objects/drawable_eigenmesh.h>
#include <cg3/meshes/eigenmesh/eigenmesh.h>
#include <Eigen/Dense>

#include <CGAL/Surface_mesh.h>
#include <cg3/cgal/cgal_aabbtree.h>
#include <cg3/cgal/cgal.h>
#include <CGAL/Cartesian/Cartesian_base.h>
#include <CGAL/Surface_mesh/IO.h>
#include <CGAL/Surface_mesh/Surface_mesh.h>
#include <CGAL/Plane_3.h>
#include <CGAL/Filtered_kernel.h>

#include <QFileDialog>
#include <QMessageBox>
#include <QStatusBar>
#include <QDebug>
#include <QFrame>

#ifdef __APPLE__
#include <gl.h>
#else
#include <GL/gl.h>
#endif

namespace Ui {
    class FourAxisMillingManager;
}

class FourAxisMillingManager : public QFrame
{
        Q_OBJECT

    public:
        explicit FourAxisMillingManager(QWidget *parent = 0);
        ~FourAxisMillingManager();


private slots:

        void on_checkPushButton_clicked();

        void on_saveMeshAxis_clicked();

        void on_loadMeshPushButton_clicked();

        void on_plusXButton_clicked();

        void on_minusXButton_clicked();

        void on_plusYButton_clicked();

        void on_minusYButton_clicked();

        void on_plusZButton_clicked();

        void on_minusZButton_clicked();

        void on_rotateButton_clicked();

        void on_scalePushButton_clicked();

        void on_inverseScalePushButton_clicked();

        void on_meshToOriginPushButton_clicked();

        void on_clearMeshPushButton_clicked();

        void on_automaticOrientationPushButton_clicked();

        void on_cutExtremesPushButton_clicked();

    private:

        cg3::DrawableEigenMesh originalMesh;
        cg3::DrawableEigenMesh smoothedMesh;
        bool loaded;
        std::vector<unsigned int> minExtreme, maxExtreme;
        Ui::FourAxisMillingManager*    ui;
        cg3::viewer::MainWindow& mainWindow;
        cg3::viewer::LoaderSaver objls;

};

#endif // DRAWMANAGER_H
