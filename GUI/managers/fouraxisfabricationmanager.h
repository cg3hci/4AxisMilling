/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#ifndef FOURAXISFABRICATIONMANAGER_H
#define FOURAXISFABRICATIONMANAGER_H


#include <cg3/viewer/mainwindow.h>
#include <cg3/viewer/utilities/loadersaver.h>

#include <cg3/viewer/interfaces/drawable_object.h>
#include <cg3/viewer/drawable_objects/drawable_eigenmesh.h>

#include <cg3/data_structures/arrays/array2d.h>

#include <QFileDialog>
#include <QMessageBox>
#include <QStatusBar>
#include <QDebug>
#include <QFrame>

#include "methods/fouraxisfabrication.h"

namespace Ui {
class FourAxisFabricationManager;
}

class FourAxisFabricationManager : public QFrame
{
    Q_OBJECT

public:

    /* Constructor/Destructors */

    explicit FourAxisFabricationManager(QWidget *parent = 0);
    ~FourAxisFabricationManager();


private:

    /* UI Fields */

    Ui::FourAxisFabricationManager* ui;
    cg3::viewer::MainWindow& mainWindow;

    cg3::viewer::LoaderSaver loaderSaverObj;

    std::string loadedMeshFile;
    std::string loadedSmoothedMeshFile;

    bool isMeshLoaded;
    bool isMeshOriented;
    bool areExtremesCut;
    bool isVisibilityChecked;
    bool areTargetDirectionsFound;
    bool isAssociationComputed;


    /* Data fields */

    cg3::DrawableEigenMesh originalMesh;
    cg3::DrawableEigenMesh smoothedMesh;

    FourAxisFabrication::Data data;

    /* UI methods */

    void initialize();
    void updateUI();    
    void clearData();

    /* Computing methods */

    void computeEntireAlgorithm();

    void optimalOrientation();
    void cutExtremes();
    void checkVisibility();
    void getTargetDirections();
    void getAssociation();


    /* Visualization methods */

    void resetCameraDirection();
    void setCameraDirection(cg3::Vec3 dir);

    void initializeVisualizationSlider();
    void updateVisualization();

    void visualizeMesh();
    void visualizeExtremes();
    void visualizeVisibility();
    void visualizeTargetDirections();
    void visualizeAssociation();


private slots:

    /* UI slots Mesh */

    void on_loadMeshButton_clicked();
    void on_clearMeshButton_clicked();
    void on_reloadMeshButton_clicked();


    /* UI slots Four Axis Fabrication */

    void on_computeEntireAlgorithmButton_clicked();

    void on_optimalOrientationButton_clicked();
    void on_cutExtremesButton_clicked();
    void on_checkVisibilityButton_clicked();
    void on_targetDirectionsButton_clicked();
    void on_getAssociationButton_clicked();


    /* UI slots Transformations */

    void on_centerOnOriginButton_clicked();

    void on_plusXButton_clicked();
    void on_minusXButton_clicked();
    void on_plusYButton_clicked();
    void on_minusYButton_clicked();
    void on_plusZButton_clicked();
    void on_minusZButton_clicked();

    void on_rotateButton_clicked();

    void on_scaleButton_clicked();

    void on_inverseScaleButton_clicked();


    /* UI slots Visulization */

    void on_meshRadio_clicked();
    void on_extremesRadio_clicked();
    void on_visibilityRadio_clicked();
    void on_targetDirectionsRadio_clicked();
    void on_associationRadio_clicked();

    void on_visualizationSlider_valueChanged(int value);
    void on_resetCameraButton_clicked();
};

#endif // FOURAXISFABRICATIONMANAGER_H
