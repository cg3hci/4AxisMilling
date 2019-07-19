/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#include "fafsegmentationmanager.h"
#include "ui_fafsegmentationmanager.h"

#include <sstream>
#include <string>
#include <fstream>
#include <set>

#include <cg3/geometry/transformations3.h>

#include <cg3/utilities/string.h>
#include <cg3/utilities/timer.h>

#include <cg3/libigl/mesh_distance.h>



/* ----- CONSTRUCTORS/DESTRUCTOR ------ */

/**
 * @brief Default constructor
 * @param parent Parent widget
 */
FAFSegmentationManager::FAFSegmentationManager(QWidget *parent) :
    QFrame(parent),
    ui(new Ui::FAFSegmentationManager),
    mainWindow((cg3::viewer::MainWindow&)*parent)
{
    ui->setupUi(this);

    initialize();
}


/**
 * @brief Destructor
 */
FAFSegmentationManager::~FAFSegmentationManager(){
    delete ui;
}




/* ----- UI METHODS ------ */

/**
 * @brief Initialization of the manager
 */
void FAFSegmentationManager::initialize() {
    loaderSaverObj.addSupportedExtension("obj");
    loaderSaverData.addSupportedExtension("faf");

    mainWindow.canvas.setOrthographicCamera();

    clearData();

    updateUI();
}

/**
 * @brief Update UI depending on the current state
 */
void FAFSegmentationManager::updateUI() {
    // ----- Mesh loading -----
    ui->loadMeshButton->setEnabled(!isMeshLoaded);
    ui->clearMeshButton->setEnabled(isMeshLoaded);
    ui->reloadMeshButton->setEnabled(isMeshLoaded);
    ui->saveResultsButton->setEnabled(isMeshLoaded);


    // ----- Four axis fabrication -----
    ui->segmentationGroup->setEnabled(isMeshLoaded);

    //Scale and stock generation
    ui->scaleCheckBox->setEnabled(!isMeshSegmented);
    ui->scaleSpinBox->setEnabled(!isMeshSegmented);
    ui->segmentationButton->setEnabled(!isMeshSegmented);
}

/**
 * @brief Clear data of four axis fabrication
 */
void FAFSegmentationManager::clearData() {
    isMeshLoaded = false;
    isMeshSegmented = false;
    mesh.clear();
    segmentedMesh.clear();
    association.clear();
    results.clear();
    resultAssociation.clear();
}

/* ----- COMPUTING METHODS ------ */

/**
 * @brief Scale mesh and segmentation
 */
void FAFSegmentationManager::segmentation() {
    if (!isMeshSegmented) {
        std::cout << std::endl << "#######################################################################" << std::endl << std::endl;

        segmentedMesh = mesh;

        //Get UI data
        bool scaleModel = ui->scaleCheckBox->isChecked();
        double modelLength = ui->scaleSpinBox->value();

        cg3::Timer t(std::string("Scale and segmentation"));

        //Scale mesh and get stock
        FourAxisFabrication::centerAndScale(
                    segmentedMesh,
                    scaleModel,
                    modelLength);

        //Segmentation
        association = FourAxisFabrication::segmentation(
                    segmentedMesh);

        t.stopAndPrint();

        isMeshSegmented = true;

        addDrawableScaledMesh();
    }
}


/* ----- VISUALIZATION METHODS ------ */

/**
 * @brief Add drawable meshes
 */
void FAFSegmentationManager::addDrawableMesh() {
    //Add drawable meshes to the canvas
    drawableMesh = cg3::DrawableEigenMesh(mesh);
    drawableMesh.setFlatShading();

    mainWindow.pushDrawableObject(&drawableMesh, "Mesh");
}

/**
 * @brief Add drawable smoothed mesh
 */
void FAFSegmentationManager::addDrawableScaledMesh() {
    drawableScaledMesh = cg3::DrawableEigenMesh(segmentedMesh);
    drawableScaledMesh.setFlatShading();
    drawableScaledMesh.setEnableVertexColor();

    mainWindow.pushDrawableObject(&drawableScaledMesh, "Segmented mesh");

    mainWindow.setDrawableObjectVisibility(&drawableMesh, false);
}

/**
 * @brief Add drawable results
 */
void FAFSegmentationManager::addDrawableResults() {
    //Hide the scaled mesh
    mainWindow.setDrawableObjectVisibility(&drawableScaledMesh, false);


    //Draw results
    drawableResults.clear();
    drawableResults.resize(results.size());
    for (size_t i = 0; i < results.size(); i++) {
        drawableResults[i] = cg3::DrawableEigenMesh(results[i]);
        drawableResults[i].setFlatShading();

        mainWindow.pushDrawableObject(&drawableResults[i], "Result " + std::to_string(i), (i == 0 ? true : false));
    }
}

/**
 * @brief Delte all drawable objects from the canvas
 */
void FAFSegmentationManager::deleteDrawableObjects() {
    if (isMeshLoaded) {
        //Delete meshes
        mainWindow.deleteDrawableObject(&drawableMesh);
        drawableMesh.clear();

        if (isMeshSegmented) {
            mainWindow.deleteDrawableObject(&drawableScaledMesh);
            drawableScaledMesh.clear();

            //Delete results
            for (cg3::DrawableEigenMesh& res : drawableResults) {
                mainWindow.deleteDrawableObject(&res);
            }
            drawableResults.clear();
        }
    }
}

/**
 * @brief Colorize mesh to the default color
 */
void FAFSegmentationManager::colorizeMesh() {
    //Variables for colors
    cg3::Color color;

    //Set the color
    drawableScaledMesh.setFaceColor(cg3::Color(128,128,128));

    std::set<int> set(association.begin(), association.end());

    if (!set.empty()) {
        int subd = 255 / set.size();

        //For each face of the drawable mesh
        for (unsigned int faceId = 0; faceId < drawableScaledMesh.numberFaces(); faceId++) {
            //Get direction index associated to the current face
            int associatedDirectionIndex = association[faceId];

            //If it has an associated fabrication direction
            if (associatedDirectionIndex >= 0) {
                color.setHsv(subd * associatedDirectionIndex, 255, 255);

                //Set the color
                drawableScaledMesh.setFaceColor(color, faceId);
            }
        }

    }
}

/**
 * @brief Colorize mesh to the default color
 */
void FAFSegmentationManager::colorizeMeshVertices() {
    //Variables for colors
    cg3::Color color;

    //Set the color
    drawableScaledMesh.setVertexColor(cg3::Color(128,128,128));

    std::set<int> set(association.begin(), association.end());

    if (!set.empty()) {
        int subd = 255 / set.size();

        //For each face of the drawable mesh
        for (unsigned int vId = 0; vId < drawableScaledMesh.numberVertices(); vId++) {
            //Get direction index associated to the current face
            int associatedDirectionIndex = association[vId];

            //If it has an associated fabrication direction
            if (associatedDirectionIndex >= 0) {
                color.setHsv(subd * associatedDirectionIndex, 255, 255);

                //Set the color
                drawableScaledMesh.setVertexColor(color, vId);
            }
        }

    }
}



/* ----- UI SLOTS MESH ------ */

void FAFSegmentationManager::on_segmentationButton_clicked() {
    //Get segmentation
    segmentation();

    //Update canvas and fit the scene
    mainWindow.canvas.update();
    mainWindow.canvas.fitScene();

    updateUI();
    colorizeMeshVertices();
}

void FAFSegmentationManager::on_loadMeshButton_clicked()
{
    if (!isMeshLoaded) {
        std::string meshFile;

        //Get loading dialog
        meshFile = loaderSaverObj.loadDialog("Load mesh");

        if (meshFile != "") {
            isMeshLoaded = mesh.loadFromObj(meshFile);

            //If the mesh has been successfully loaded
            if (isMeshLoaded){
                std::string rawname, ext;
                cg3::separateExtensionFromFilename(meshFile, rawname, ext);

                std::string meshName = meshFile.substr(meshFile.find_last_of("/") + 1);

                addDrawableMesh();

                //Update canvas and fit the scene
                mainWindow.canvas.update();
                mainWindow.canvas.fitScene();

                std::cout << std::endl;

                std::cout << "Original: \"" << meshName << "\" " << "(" << mesh.numberFaces() << " F / " << mesh.numberVertices() << " V)" << std::endl;
            }
            else {
                clearData();
            }

        }

        updateUI();
    }
}

void FAFSegmentationManager::on_clearMeshButton_clicked()
{
    if (isMeshLoaded) {
        //Delete objects from the canvas
        deleteDrawableObjects();

        //Clear four axis fabrication data
        clearData();

        //Update canvas and fit the scene
        mainWindow.canvas.update();
        mainWindow.canvas.fitScene();
        updateUI();
    }

}

void FAFSegmentationManager::on_reloadMeshButton_clicked()
{
    if (isMeshLoaded) {
        //Delete drawable Objects
        deleteDrawableObjects();

        //Saving for reloading
        cg3::EigenMesh originalMeshCopy = mesh;

        //Clear four axis fabrication data
        clearData();

        //Reload
        mesh = originalMeshCopy;

        isMeshLoaded = true;

        //Add drawable meshes
        addDrawableMesh();

        //Update canvas and fit the scene
        mainWindow.canvas.update();
        mainWindow.canvas.fitScene();

        updateUI();
    }
}

void FAFSegmentationManager::on_saveResultsButton_clicked() {
    //Get saving dialog
    std::string selectedExtension;
    std::string saveFileName = loaderSaverObj.saveDialog("Save mesh", selectedExtension);
    saveFileName += "." + selectedExtension;

    if (saveFileName != "") {
        std::string rawname, ext;
        cg3::separateExtensionFromFilename(saveFileName, rawname, ext);

        //Save on obj files
        mesh.saveOnObj(rawname + "_original.obj");
        if (isMeshSegmented) {
            segmentedMesh.saveOnObj(rawname + "_segmentation.obj");

            for (size_t i = 0; i < results.size(); i++) {
                cg3::EigenMesh& mesh = results[i];
                mesh.setVertexColor(128,128,128);
                mesh.saveOnObj(rawname + "_result_" + std::to_string(i) + ".obj");
            }
        }
    }
}
