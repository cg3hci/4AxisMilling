/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#ifndef FAFMANAGER_H
#define FAFMANAGER_H


#include <cg3/viewer/mainwindow.h>
#include <cg3/viewer/utilities/loadersaver.h>

#include <cg3/viewer/interfaces/drawable_object.h>
#include <cg3/viewer/interfaces/drawable_container.h>
#include <cg3/viewer/drawable_objects/drawable_eigenmesh.h>
#include <cg3/viewer/pickable_objects/pickable_eigenmesh.h>

#include <cg3/data_structures/arrays/array2d.h>

#include <cg3/libigl/geodesics.h>

#include <QFileDialog>
#include <QMessageBox>
#include <QStatusBar>
#include <QDebug>
#include <QFrame>

#include "../methods/fouraxisfabrication.h"

namespace Ui {
class FAFManager;
}

class FAFManager : public QFrame
{
    Q_OBJECT

public:

    /* Constructor/Destructors */

    explicit FAFManager(QWidget *parent = nullptr);
    ~FAFManager();


private:    

    /* Data fields */

    FourAxisFabrication::Data data;


    /* Drawable objects */

    cg3::DrawableEigenMesh drawableOriginalMesh;

    cg3::PickableEigenMesh drawableDetailMesh;
//    cg3::libigl::HeatGeodesicsData detailMeshGeodesicsData;

    cg3::DrawableEigenMesh drawableSmoothedMesh;
    cg3::DrawableEigenMesh drawableStock;

    cg3::DrawableEigenMesh drawableRestoredMesh;

    cg3::DrawableEigenMesh drawableMinComponent;
    cg3::DrawableEigenMesh drawableMaxComponent;
    cg3::DrawableEigenMesh drawableFourAxisComponent;

    cg3::DrawableEigenMesh drawableMinResult;
    cg3::DrawableEigenMesh drawableMaxResult;

    std::vector<cg3::DrawableEigenMesh> drawableBoxes;
    std::vector<cg3::DrawableEigenMesh> drawableStocks;
    std::vector<cg3::DrawableEigenMesh> drawableResults;

    cg3::DrawableEigenMesh drawableMinSupport;
    cg3::DrawableEigenMesh drawableMaxSupport;


    /* UI Fields */

    Ui::FAFManager* ui;
    cg3::viewer::MainWindow& mainWindow;

    cg3::viewer::LoaderSaver loaderSaverObj;
    cg3::viewer::LoaderSaver loaderSaverData;


    /* UI methods */

    void initialize();
    void updateUI();    
    void clearData();

    /* Computing methods */

    void scaleAndStock();
    void findDetails();
    void smoothing();
    void optimalOrientation();
    void selectExtremes();
    void checkVisibility();
    void getAssociation();
    void optimizeAssociation();
    void smoothLines();
    void restoreFrequencies();
    void cutComponents();
    void extractResults();


    /* Visualization methods */

    void addDrawableMesh();
    void addDrawableStock();
    void addDrawableDetailMesh();
    void addDrawableSmoothedMesh();
    void addDrawableRestoredMesh();
    void addDrawableCutComponents();
    void addDrawableResults();
    void updateDrawableMesh();
    void updateDrawableSmoothedMesh();
    void updateDrawableRestoredMesh();

    void deleteDrawableObjects();

    void resetCameraDirection();
    void setCameraDirection(const cg3::Vec3d& dir);

    void initializeVisualizationSlider();
    void updateVisualization();

    void colorizeMesh();
    void colorizeDetailMesh();
    void colorizeExtremes();
    void colorizeVisibility();
    void colorizeAssociation();
    void colorizeAssociation(
            cg3::DrawableEigenMesh& drawableMesh,
            const std::vector<int>& association,
            const std::vector<unsigned int>& targetDirections,
            const std::vector<unsigned int>& nonVisibleFaces);
    void showResults();
    void showCurrentStatusDescription();

    cg3::Color computeColorByNormalizedValue(const double value);


private slots:

    /* UI slots Mesh */

    void on_loadMeshButton_clicked();
    void on_clearMeshButton_clicked();
    void on_reloadMeshButton_clicked();
    void on_saveResultsButton_clicked();

    /* UI slots for saving/loading data */

    void on_loadDataButton_clicked();
    void on_saveDataButton_clicked();


    /* UI slots Four Axis Fabrication */

    void on_scaleStockButton_clicked();
    void on_saliencyFindDetailsButton_clicked();
    void on_smoothingButton_clicked();
    void on_optimalOrientationButton_clicked();
    void on_selectExtremesButton_clicked();
    void on_checkVisibilityButton_clicked();
    void on_getAssociationButton_clicked();
    void on_optimizationButton_clicked();
    void on_smoothLinesButton_clicked();
    void on_restoreFrequenciesButton_clicked();
    void on_cutComponentsButton_clicked();
    void on_extractResultsButton_clicked();

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
    void on_associationRadio_clicked();
    void on_resultsRadio_clicked();

    void on_showNonVisibleCheck_clicked();
    void on_resetCameraButton_clicked();
    void on_visualizationSlider_valueChanged(int value);

    void meshPainted();
    void facePicked(const cg3::PickableObject* obj, unsigned int f);

    void on_generateResultsButton_clicked();
};

#endif // FAFMANAGER_H
