/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#ifndef FAFSEGMENTATIONMANAGER_H
#define FAFSEGMENTATIONMANAGER_H


#include <cg3/viewer/mainwindow.h>
#include <cg3/viewer/utilities/loadersaver.h>

#include <cg3/viewer/interfaces/drawable_object.h>
#include <cg3/viewer/interfaces/drawable_container.h>
#include <cg3/viewer/drawable_objects/drawable_eigenmesh.h>

#include <QFileDialog>
#include <QMessageBox>
#include <QStatusBar>
#include <QDebug>
#include <QFrame>

#include "../methods/faf/faf_split.h"
#include "../methods/faf/faf_various.h"

namespace Ui {
class FAFSegmentationManager;
}

class FAFSegmentationManager : public QFrame
{
    Q_OBJECT

public:

    /* Constructor/Destructors */

    explicit FAFSegmentationManager(QWidget *parent = nullptr);
    ~FAFSegmentationManager();


private:

    /* Objects */
    cg3::EigenMesh mesh;
    cg3::EigenMesh segmentedMesh;
    std::vector<int> association;
    std::vector<cg3::EigenMesh> results;
    std::vector<int> resultAssociation;

    bool isMeshLoaded;
    bool isMeshSegmented;

    /* Drawable objects */
    cg3::DrawableEigenMesh drawableMesh;
    cg3::DrawableEigenMesh drawableScaledMesh;
    std::vector<cg3::DrawableEigenMesh> drawableResults;


    /* UI Fields */

    Ui::FAFSegmentationManager* ui;
    cg3::viewer::MainWindow& mainWindow;

    cg3::viewer::LoaderSaver loaderSaverObj;
    cg3::viewer::LoaderSaver loaderSaverData;


    /* UI methods */

    void initialize();
    void updateUI();    
    void clearData();


    /* Computing methods */
    void segmentation();


    /* Visualization methods */

    void addDrawableMesh();
    void addDrawableScaledMesh();
    void addDrawableResults();

    void deleteDrawableObjects();

    void colorizeMesh();
    void colorizeMeshVertices();


private slots:

    /* UI slots Mesh */
    void on_segmentationButton_clicked();

    void on_loadMeshButton_clicked();
    void on_clearMeshButton_clicked();
    void on_reloadMeshButton_clicked();
    void on_saveResultsButton_clicked();
};

#endif // FAFSEGMENTATIONMANAGER_H
