/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#include "fouraxisfabricationmanager.h"
#include "ui_fouraxisfabricationmanager.h"

#include <sstream>
#include <string>
#include <fstream>

#include <cg3/geometry/transformations.h>

#include <cg3/utilities/string.h>
#include <cg3/utilities/timer.h>

#include <cg3/libigl/mesh_distance.h>



/* ----- CONSTRUCTORS/DESTRUCTOR ------ */

/**
 * @brief Default constructor
 * @param parent Parent widget
 */
FourAxisFabricationManager::FourAxisFabricationManager(QWidget *parent) :
    QFrame(parent),
    ui(new Ui::FourAxisFabricationManager),
    mainWindow((cg3::viewer::MainWindow&)*parent)
{
    ui->setupUi(this);

    initialize();
}


/**
 * @brief Destructor
 */
FourAxisFabricationManager::~FourAxisFabricationManager(){
    delete ui;
}




/* ----- UI METHODS ------ */

/**
 * @brief Initialization of the manager
 */
void FourAxisFabricationManager::initialize() {
    loaderSaverObj.addSupportedExtension("obj");
    loaderSaverData.addSupportedExtension("faf");

    mainWindow.canvas.setOrthographicCamera();

    clearData();

    updateUI();
}

/**
 * @brief Update UI depending on the current state
 */
void FourAxisFabricationManager::updateUI() {
    // ----- Mesh loading -----
    ui->loadMeshButton->setEnabled(!data.isMeshLoaded);
    ui->clearMeshButton->setEnabled(data.isMeshLoaded);
    ui->reloadMeshButton->setEnabled(data.isMeshLoaded);
    ui->saveResultsButton->setEnabled(data.isMeshLoaded);
    ui->loadDataButton->setEnabled(!data.isMeshLoaded);
    ui->saveDataButton->setEnabled(data.isMeshLoaded);


    // ----- Four axis fabrication -----
    ui->fourAxisFabricationGroup->setEnabled(data.isMeshLoaded);

    //Optimal orientation
    ui->optimalOrientationButton->setEnabled(!data.isMeshOriented);
    ui->optimalOrientationOrientationsLabel->setEnabled(!data.isMeshOriented);
    ui->optimalOrientationOrientationsSpinBox->setEnabled(!data.isMeshOriented);
    ui->optimalOrientationDeterministicCheckBox->setEnabled(!data.isMeshOriented);
    ui->optimalOrientationDeepnessWeightLabel->setEnabled(!data.isMeshOriented);
    ui->optimalOrientationDeepnessWeightSpinBox->setEnabled(!data.isMeshOriented);
    ui->optimalOrientationOffsetWeightLabel->setEnabled(!data.isMeshOriented);
    ui->optimalOrientationOffsetWeightSpinBox->setEnabled(!data.isMeshOriented);

    //Select extremes
    ui->selectExtremesButton->setEnabled(!data.areExtremesSelected);
    ui->selectExtremesHeightfieldAngleLabel->setEnabled(!data.areExtremesSelected);
    ui->selectExtremesHeightfieldAngleSpinBox->setEnabled(!data.areExtremesSelected);

    //Check visibility
    ui->checkVisibilityButton->setEnabled(!data.isVisibilityChecked);
    ui->checkVisibilityDirectionsLabel->setEnabled(!data.isVisibilityChecked);
    ui->checkVisibilityDirectionsSpinBox->setEnabled(!data.isVisibilityChecked);
    ui->checkVisibilityResolutionLabel->setEnabled(!data.isVisibilityChecked);
    ui->checkVisibilityResolutionSpinBox->setEnabled(!data.isVisibilityChecked);
    ui->checkVisibilityMethodFrame->setEnabled(!data.isVisibilityChecked);
    ui->checkVisibilityXDirectionsCheckBox->setEnabled(!data.isVisibilityChecked);

    //Get association
    ui->getAssociationButton->setEnabled(!data.isAssociationComputed);
    ui->getAssociationDataSigmaSpinBox->setEnabled(!data.isAssociationComputed);
    ui->getAssociationDataSigmaLabel->setEnabled(!data.isAssociationComputed);
    ui->getAssociationFreeCostAngleSpinBox->setEnabled(!data.isAssociationComputed);
    ui->getAssociationFreeCostAngleLabel->setEnabled(!data.isAssociationComputed);    
    ui->getAssociationCompactnessLabel->setEnabled(!data.isAssociationComputed);
    ui->getAssociationCompactnessSpinBox->setEnabled(!data.isAssociationComputed);
    ui->getAssociationFixExtremesCheckBox->setEnabled(!data.isAssociationComputed);

    //Optimization
    ui->optimizationButton->setEnabled(!data.isAssociationOptimized);
    ui->optimizationRelaxHolesCheckBox->setEnabled(!data.isAssociationOptimized);
    ui->optimizationLoseHolesCheckBox->setEnabled(!data.isAssociationOptimized);
    ui->optimizationMinChartAreaLabel->setEnabled(!data.isAssociationOptimized);
    ui->optimizationMinChartAreaSpinBox->setEnabled(!data.isAssociationOptimized);

    //Restore frequencies
    ui->restoreFrequenciesButton->setEnabled(!data.areFrequenciesRestored);
    ui->restoreFrequenciesIterationsLabel->setEnabled(!data.areFrequenciesRestored);
    ui->restoreFrequenciesIterationsSpinBox->setEnabled(!data.areFrequenciesRestored);

    //Recheck visibility after restore
    ui->recheckVisibilityReassignNonVisibleCheckBox->setEnabled(!data.isVisibilityRecheckedAfterRestore);
    ui->recheckVisibilityButton->setEnabled(!data.isVisibilityRecheckedAfterRestore);

    //Cut components
    ui->cutComponentsButton->setEnabled(!data.areComponentsCut);
    ui->cutComponentsCheckBox->setEnabled(!data.areComponentsCut);

    //Extract results
    ui->extractResultsButton->setEnabled(!data.areResultsExtracted);
    ui->extractResultsStockLengthLabel->setEnabled(!data.areResultsExtracted);
    ui->extractResultsStockLengthSpinBox->setEnabled(!data.areResultsExtracted);
    ui->extractResultsStockDiameterLabel->setEnabled(!data.areResultsExtracted);
    ui->extractResultsStockDiameterSpinBox->setEnabled(!data.areResultsExtracted);
    ui->extractResultsModelLengthLabel->setEnabled(!data.areResultsExtracted);
    ui->extractResultsModelLengthSpinBox->setEnabled(!data.areResultsExtracted);
    ui->extractResultsFirstLayerAngleLabel->setEnabled(!data.areResultsExtracted);
    ui->extractResultsFirstLayerAngleSpinBox->setEnabled(!data.areResultsExtracted);
    ui->extractResultsSecondLayerAngleLabel->setEnabled(!data.areResultsExtracted);
    ui->extractResultsSecondLayerAngleSpinBox->setEnabled(!data.areResultsExtracted);
    ui->extractResultsFirstLayerHeightLabel->setEnabled(!data.areResultsExtracted);
    ui->extractResultsFirstLayerHeightSpinBox->setEnabled(!data.areResultsExtracted);
    ui->extractResultsXDirectionsOrderLabel->setEnabled(!data.areResultsExtracted);
    ui->extractResultsXDirectionsOrderFrame->setEnabled(!data.areResultsExtracted);
    ui->extractResultsRotateCheckBox->setEnabled(!data.areResultsExtracted);




    // ----- Visualization -----
    ui->visualizationGroup->setEnabled(data.isMeshLoaded);

    //Radio
    ui->meshRadio->setEnabled(data.isMeshLoaded);
    ui->extremesRadio->setEnabled(data.areExtremesSelected);
    ui->visibilityRadio->setEnabled(data.isVisibilityChecked);
    ui->associationRadio->setEnabled(data.isAssociationComputed);
    ui->resultsRadio->setEnabled(data.areResultsExtracted);


    // ----- Transformations -----
    ui->transformationGroup->setEnabled(data.isMeshLoaded);
}

/**
 * @brief Clear data of four axis fabrication
 */
void FourAxisFabricationManager::clearData() {
    data.clear();
}




/* ----- COMPUTING METHODS ------ */

/**
 * @brief Compute optimal orientation
 */
void FourAxisFabricationManager::optimalOrientation() {
    if (!data.isMeshOriented) {
        std::cout << std::endl << "#######################################################################" << std::endl << std::endl;

        //Get UI data
        unsigned int nOrientations = (unsigned int) ui->optimalOrientationOrientationsSpinBox->value();
        double deepnessWeight = (double) ui->optimalOrientationDeepnessWeightSpinBox->value();
        double offsetWeight = (double) ui->optimalOrientationOffsetWeightSpinBox->value();
        bool deterministic = ui->optimalOrientationDeterministicCheckBox->isChecked();

        cg3::Timer t("Optimal orientation");

        //Get optimal mesh orientation
        FourAxisFabrication::rotateToOptimalOrientation(
                    data.mesh,
                    data.smoothedMesh,
                    nOrientations,
                    deepnessWeight,
                    offsetWeight,
                    deterministic);

        t.stopAndPrint();

        data.isMeshOriented = true;

        updateDrawableMeshes();
    }
}

/**
 * @brief Select extremes
 */
void FourAxisFabricationManager::selectExtremes() {
    if (!data.areExtremesSelected) {
        optimalOrientation();

        //Get UI data
        double heightfieldAngle = ui->selectExtremesHeightfieldAngleSpinBox->value() / 180.0 * M_PI;

        cg3::Timer t("Select extremes");

        //Get extremes on x-axis to be selected
        FourAxisFabrication::selectExtremesOnXAxis(data.smoothedMesh, heightfieldAngle, data);

        t.stopAndPrint();


        data.areExtremesSelected = true;
    }
}

/**
 * @brief Check visibility from various directions
 */
void FourAxisFabricationManager::checkVisibility() {
    if (!data.isVisibilityChecked) {
        selectExtremes();

        //Get UI data
        double heightfieldAngle = ui->selectExtremesHeightfieldAngleSpinBox->value() / 180.0 * M_PI;
        unsigned int nDirections = (unsigned int) ui->checkVisibilityDirectionsSpinBox->value();
        unsigned int resolution = (unsigned int) ui->checkVisibilityResolutionSpinBox->value();
        bool includeXDirections = ui->checkVisibilityXDirectionsCheckBox->isChecked();
        FourAxisFabrication::CheckMode checkMode =
                (ui->checkVisibilityGLRadio->isChecked() ?
                     FourAxisFabrication::OPENGL :
                     ui->checkVisibilityRayRadio->isChecked() ?
                         FourAxisFabrication::RAYSHOOTING :
                         FourAxisFabrication::PROJECTION);

        cg3::Timer t("Visibility check");

        //Visibility check
        FourAxisFabrication::getVisibility(
                    data.smoothedMesh,
                    nDirections,
                    resolution,
                    heightfieldAngle,
                    includeXDirections,
                    data,
                    checkMode);

        t.stopAndPrint();

        data.isVisibilityChecked = true;

        std::cout << "Non-visible triangles: " << data.nonVisibleFaces.size() << std::endl;
    }
}

/**
 * @brief Get association
 */
void FourAxisFabricationManager::getAssociation() {
    if (!data.isAssociationComputed) {
        checkVisibility();

        //Get UI data
        double freeCostAngle = ui->getAssociationFreeCostAngleSpinBox->value() / 180.0 * M_PI;
        double dataSigma = ui->getAssociationDataSigmaSpinBox->value();
        double compactness = ui->getAssociationCompactnessSpinBox->value();
        bool fixExtremes = ui->getAssociationFixExtremesCheckBox->isChecked();

        cg3::Timer t("Get association");

        //Get association
        FourAxisFabrication::getAssociation(
                    data.smoothedMesh,
                    freeCostAngle,
                    dataSigma,
                    compactness,
                    fixExtremes,
                    data);

        t.stopAndPrint();

        data.isAssociationComputed = true;
    }
}


/**
 * @brief Get optimized association
 */
void FourAxisFabricationManager::optimizeAssociation() {
    if (!data.isAssociationOptimized) {
        getAssociation();

        //Get UI data
        bool relaxHoles = ui->optimizationRelaxHolesCheckBox->isChecked();
        bool loseHoles = ui->optimizationLoseHolesCheckBox->isChecked();
        double minChartArea = ui->optimizationMinChartAreaSpinBox->value();

        cg3::Timer t("Optimize association");

        //Execute optimization
        FourAxisFabrication::optimization(
                    data.smoothedMesh,
                    relaxHoles,
                    loseHoles,
                    minChartArea,
                    data);

        t.stopAndPrint();

        data.isAssociationOptimized = true;

        std::cout << "Non-visible triangles after association: " << data.associationNonVisibleFaces.size() << std::endl;
    }
}


/**
 * @brief Restore frequencies
 */
void FourAxisFabricationManager::restoreFrequencies() {
    if (!data.areFrequenciesRestored) {
        optimizeAssociation();

        //Get UI data
        double heightfieldAngle = ui->selectExtremesHeightfieldAngleSpinBox->value() / 180.0 * M_PI;
        unsigned int nIterations = (unsigned int) ui->restoreFrequenciesIterationsSpinBox->value();

        double haussDistance = cg3::libigl::hausdorffDistance(data.mesh, data.smoothedMesh);
        cg3::BoundingBox originalMeshBB = data.mesh.boundingBox();
        double haussDistanceBB = haussDistance/originalMeshBB.diag();

        std::cout << "Smoothed -> Haussdorff distance: " << haussDistance << " (w.r.t. bounding box: " << haussDistanceBB << ")" << std::endl;

        cg3::Timer t("Restore frequencies");

        //Restore frequencies
        FourAxisFabrication::restoreFrequencies(nIterations, heightfieldAngle, data.mesh, data.smoothedMesh, data);

        t.stopAndPrint();

        haussDistance = cg3::libigl::hausdorffDistance(data.mesh, data.restoredMesh);
        originalMeshBB = data.mesh.boundingBox();
        haussDistanceBB = haussDistance/originalMeshBB.diag();

        std::cout << "Restored -> Haussdorff distance: " << haussDistance << " (w.r.t. bounding box: " << haussDistanceBB << ")" << std::endl;

        data.areFrequenciesRestored = true;

        addDrawableRestoredMesh();
    }
}


/**
 * @brief Recheck visibility after frequencies are restored
 */
void FourAxisFabricationManager::recheckVisibilityAfterRestore() {
    if (!data.isVisibilityRecheckedAfterRestore) {
        restoreFrequencies();

        //Get UI data
        double heightfieldAngle = ui->selectExtremesHeightfieldAngleSpinBox->value() / 180.0 * M_PI;
        unsigned int resolution = (unsigned int) ui->checkVisibilityResolutionSpinBox->value();
        bool includeXDirections = ui->checkVisibilityXDirectionsCheckBox->isChecked();
        bool reassign = ui->recheckVisibilityReassignNonVisibleCheckBox->isChecked();
        FourAxisFabrication::CheckMode checkMode =
                (ui->checkVisibilityGLRadio->isChecked() ?
                     FourAxisFabrication::OPENGL :
                     ui->checkVisibilityRayRadio->isChecked() ?
                         FourAxisFabrication::RAYSHOOTING :
                         FourAxisFabrication::PROJECTION);


        cg3::Timer tCheck("Recheck visibility after frequencies have been restored");

        //Check if it is a valid association
        FourAxisFabrication::recheckVisibilityAfterRestore(resolution, heightfieldAngle, includeXDirections, reassign, data, checkMode);

        tCheck.stopAndPrint();

        std::cout << "Non-visible triangles after recheck: " << data.restoredMeshNonVisibleFaces.size() << std::endl;

        data.isVisibilityRecheckedAfterRestore = true;

        updateDrawableRestoredMesh();
    }
}



/**
 * @brief Cut components
 */
void FourAxisFabricationManager::cutComponents() {
    if (!data.areComponentsCut) {
        recheckVisibilityAfterRestore();

        //Get UI data
        bool cutComponents = ui->cutComponentsCheckBox->isChecked();



        cg3::Timer t("Cut components");

        //Cut components
        FourAxisFabrication::cutComponents(data, cutComponents);

        t.stopAndPrint();

        data.areComponentsCut = true;

        addDrawableCutComponents();
    }
}

/**
 * @brief Extract results
 */
void FourAxisFabricationManager::extractResults() {
    if (!data.areResultsExtracted) {
        cutComponents();

        //Get UI data
        double modelLength = ui->extractResultsModelLengthSpinBox->value();
        double stockLength = ui->extractResultsStockLengthSpinBox->value();
        double stockDiameter = ui->extractResultsStockDiameterSpinBox->value();
        double firstLayerAngle = ui->extractResultsFirstLayerAngleSpinBox->value() / 180.0 * M_PI;
        double secondLayerAngle = ui->extractResultsSecondLayerAngleSpinBox->value() / 180.0 * M_PI;
        double firstLayerHeight = ui->extractResultsFirstLayerHeightSpinBox->value();
        bool rotateResults = ui->extractResultsRotateCheckBox->isChecked();
        bool xDirectionsAfter = ui->extractResultsXDirectionsAfterRadio->isChecked();

        cg3::Timer t("Extract results");

        //Extract results
        FourAxisFabrication::extractResults(data, modelLength, stockLength, stockDiameter, firstLayerAngle, secondLayerAngle, firstLayerHeight, xDirectionsAfter, rotateResults);

        t.stopAndPrint();

        data.areResultsExtracted = true;

        addDrawableResults();
    }
}




/* ----- VISUALIZATION METHODS ------ */

/**
 * @brief Add drawable meshes
 */
void FourAxisFabricationManager::addDrawableMeshes() {
    //Add drawable meshes to the canvas
    drawableOriginalMesh = cg3::DrawableEigenMesh(data.mesh);
    drawableOriginalMesh.setFlatShading();
    drawableSmoothedMesh = cg3::DrawableEigenMesh(data.smoothedMesh);
    drawableSmoothedMesh.setFlatShading();

    mainWindow.pushDrawableObject(&drawableOriginalMesh, "Mesh");
    mainWindow.pushDrawableObject(&drawableSmoothedMesh, "Smoothed mesh");

    mainWindow.setDrawableObjectVisibility(&drawableOriginalMesh, false);
}


/**
 * @brief Add drawable restored mesh to the canvas
 */
void FourAxisFabricationManager::addDrawableRestoredMesh() {
    //Hide smoothed mesh
    mainWindow.setDrawableObjectVisibility(&drawableSmoothedMesh, false);

    //Create drawable meshes
    drawableRestoredMesh = cg3::DrawableEigenMesh(data.restoredMesh);
    drawableRestoredMesh.setFlatShading();

    //Push in the canvas
    mainWindow.pushDrawableObject(&drawableRestoredMesh, "Restored mesh");
}


/**
 * @brief Add drawable cut components to the canvas
 */
void FourAxisFabricationManager::addDrawableCutComponents() {
    //Hide restored mesh
    mainWindow.setDrawableObjectVisibility(&drawableRestoredMesh, false);

    //Create drawable meshes and push in the canvas
    drawableFourAxisComponent = cg3::DrawableEigenMesh(data.fourAxisComponent);
    drawableFourAxisComponent.setFlatShading();
    mainWindow.pushDrawableObject(&drawableFourAxisComponent, "4-axis component");

    if (data.minComponent.numberFaces() > 0) {
        drawableMinComponent = cg3::DrawableEigenMesh(data.minComponent);
        drawableMinComponent.setFlatShading();
        mainWindow.pushDrawableObject(&drawableMinComponent, "Min component");
    }
    if (data.maxComponent.numberFaces() > 0) {
        drawableMaxComponent = cg3::DrawableEigenMesh(data.maxComponent);
        drawableMaxComponent.setFlatShading();
        mainWindow.pushDrawableObject(&drawableMaxComponent, "Max component");
    }
}

/**
 * @brief Add drawable results
 */
void FourAxisFabricationManager::addDrawableResults() {
    //Hide the cut components
    mainWindow.setDrawableObjectVisibility(&drawableMinComponent, false);
    mainWindow.setDrawableObjectVisibility(&drawableMaxComponent, false);
    mainWindow.setDrawableObjectVisibility(&drawableFourAxisComponent, false);

    //Add boxes (hidden by default)
    drawableBoxes.clear();
    drawableBoxes.resize(data.boxes.size());
    for (size_t i = 0; i < data.boxes.size(); i++) {
        drawableBoxes[i] = cg3::DrawableEigenMesh(data.boxes[i]);
        drawableBoxes[i].setFlatShading();

        mainWindow.pushDrawableObject(&drawableBoxes[i], "Box " + std::to_string(i), false);
    }

    //Add stock (hidden by default)
    drawableStocks.clear();
    drawableStocks.resize(data.stocks.size());
    for (size_t i = 0; i < data.stocks.size(); i++) {
        drawableStocks[i] = cg3::DrawableEigenMesh(data.stocks[i]);
        drawableStocks[i].setFlatShading();

        mainWindow.pushDrawableObject(&drawableStocks[i], "Stock " + std::to_string(i), false);
    }

    //Draw results
    drawableResults.clear();
    drawableResults.resize(data.results.size());
    for (size_t i = 0; i < data.results.size(); i++) {
        drawableResults[i] = cg3::DrawableEigenMesh(data.results[i]);
        drawableResults[i].setFlatShading();

        mainWindow.pushDrawableObject(&drawableResults[i], "Result " + std::to_string(i), (i == 0 ? true : false));
    }

    if (data.minResult.numberFaces() > 0) {
        drawableMinResult = cg3::DrawableEigenMesh(data.minResult);
        drawableMinResult.setFlatShading();
        mainWindow.pushDrawableObject(&drawableMinResult, "Min result", false);
    }
    if (data.maxResult.numberFaces() > 0) {
        drawableMaxResult = cg3::DrawableEigenMesh(data.maxResult);
        drawableMaxResult.setFlatShading();
        mainWindow.pushDrawableObject(&drawableMaxResult, "Max result", false);
    }

    //Draw supports
    drawableMinSupport = cg3::DrawableEigenMesh(data.minSupport);
    drawableMinSupport.setFlatShading();
    mainWindow.pushDrawableObject(&drawableMinSupport, "Min support", false);
    drawableMaxSupport = cg3::DrawableEigenMesh(data.maxSupport);
    drawableMaxSupport.setFlatShading();
    mainWindow.pushDrawableObject(&drawableMaxSupport, "Max support", false);
}

/**
 * @brief Update drawable meshes
 */
void FourAxisFabricationManager::updateDrawableMeshes() {
    //Update drawable meshes (already in the canvas)
    bool originalVisibility = drawableOriginalMesh.isVisible();
    bool smoothVisibility = drawableSmoothedMesh.isVisible();

    drawableOriginalMesh = cg3::DrawableEigenMesh(data.mesh);
    drawableOriginalMesh.setFlatShading();
    drawableSmoothedMesh = cg3::DrawableEigenMesh(data.smoothedMesh);
    drawableSmoothedMesh.setFlatShading();

    mainWindow.setDrawableObjectVisibility(&drawableOriginalMesh, originalVisibility);
    mainWindow.setDrawableObjectVisibility(&drawableSmoothedMesh, smoothVisibility);
}

/**
 * @brief Update drawable meshes
 */
void FourAxisFabricationManager::updateDrawableRestoredMesh() {
    //Update drawable meshes (already in the canvas)
    bool restoredVisibility = drawableRestoredMesh.isVisible();

    drawableRestoredMesh = cg3::DrawableEigenMesh(data.restoredMesh);
    drawableRestoredMesh.setFlatShading();

    mainWindow.setDrawableObjectVisibility(&drawableRestoredMesh, restoredVisibility);
}

/**
 * @brief Delte all drawable objects from the canvas
 */
void FourAxisFabricationManager::deleteDrawableObjects() {
    if (data.isMeshLoaded) {
        //Delete meshes
        mainWindow.deleteDrawableObject(&drawableOriginalMesh);
        mainWindow.deleteDrawableObject(&drawableSmoothedMesh);

        drawableOriginalMesh.clear();
        drawableSmoothedMesh.clear();

        if (data.areFrequenciesRestored) {
            //Delete restored mesh
            mainWindow.deleteDrawableObject(&drawableRestoredMesh);

            drawableRestoredMesh.clear();

            //Delete cut components
            if (data.areComponentsCut) {
                mainWindow.deleteDrawableObject(&drawableFourAxisComponent);                
                drawableFourAxisComponent.clear();

                if (data.minComponent.numberFaces() > 0) {
                    mainWindow.deleteDrawableObject(&drawableMinComponent);
                    drawableMinComponent.clear();
                }
                if (data.maxComponent.numberFaces() > 0) {
                    mainWindow.deleteDrawableObject(&drawableMaxComponent);
                    drawableMaxComponent.clear();
                }


                //Delete results
                if (data.areResultsExtracted) {
                    //Delete boxes
                    for (cg3::DrawableEigenMesh& box : drawableBoxes) {
                        mainWindow.deleteDrawableObject(&box);
                    }
                    drawableBoxes.clear();

                    //Delete stocks
                    for (cg3::DrawableEigenMesh& st : drawableStocks) {
                        mainWindow.deleteDrawableObject(&st);
                    }
                    drawableStocks.clear();

                    //Delete results

                    for (cg3::DrawableEigenMesh& res : drawableResults) {
                        mainWindow.deleteDrawableObject(&res);
                    }
                    drawableResults.clear();

                    if (data.minResult.numberFaces() > 0) {
                        mainWindow.deleteDrawableObject(&drawableMinResult);
                        drawableMinResult.clear();
                    }
                    if (data.maxResult.numberFaces() > 0) {
                        mainWindow.deleteDrawableObject(&drawableMaxResult);
                        drawableMaxResult.clear();
                    }

                    mainWindow.deleteDrawableObject(&drawableMinSupport);
                    drawableMinSupport.clear();
                    mainWindow.deleteDrawableObject(&drawableMaxSupport);
                    drawableMaxSupport.clear();
                }
            }

        }
    }    

}


/**
 * @brief Reset camera direction
 */
void FourAxisFabricationManager::resetCameraDirection() {
    mainWindow.canvas.setCameraDirection(cg3::Vec3(1,0,0));
    mainWindow.canvas.setCameraDirection(cg3::Vec3(0,0,-1));

    mainWindow.canvas.update();
    mainWindow.canvas.fitScene();
}

/**
 * @brief Set camera direction
 */
void FourAxisFabricationManager::setCameraDirection(const cg3::Vec3& dir) {
    mainWindow.canvas.setCameraDirection(dir);

    mainWindow.canvas.update();
    mainWindow.canvas.fitScene();
}


/**
 * @brief Initialize the slider depending on which radio button you have chosen
 */
void FourAxisFabricationManager::initializeVisualizationSlider() {
    ui->descriptionLabel->setEnabled(!ui->meshRadio->isChecked());
    ui->visualizationSlider->setEnabled(!ui->meshRadio->isChecked());
    ui->moveCameraCheckBox->setEnabled(!ui->meshRadio->isChecked());

    //Setting minimum, maximum and values of the slider
    if (ui->meshRadio->isChecked()) {
        ui->visualizationSlider->setMinimum(0);
        ui->visualizationSlider->setMaximum(0);
        ui->visualizationSlider->setValue(0);
        ui->showNonVisibleCheck->setEnabled(false);
    }
    else if (ui->extremesRadio->isChecked()) {
        ui->visualizationSlider->setMinimum(0);
        ui->visualizationSlider->setMaximum(2);
        ui->visualizationSlider->setValue(0);
        ui->showNonVisibleCheck->setEnabled(false);
    }
    else if (ui->visibilityRadio->isChecked()) {
        ui->visualizationSlider->setMinimum(0);
        ui->visualizationSlider->setMaximum(data.visibility.sizeX());
        ui->visualizationSlider->setValue(0);
        ui->showNonVisibleCheck->setEnabled(true);
    }    
    else if (ui->associationRadio->isChecked()) {
        ui->visualizationSlider->setMinimum(0);
        ui->visualizationSlider->setMaximum(data.targetDirections.size());
        ui->visualizationSlider->setValue(0);
        ui->showNonVisibleCheck->setEnabled(true);
    }
    else if (ui->resultsRadio->isChecked()) {
        ui->visualizationSlider->setMinimum(0);
        ui->visualizationSlider->setMaximum(data.results.size()-1);
        ui->visualizationSlider->setValue(0);
        ui->showNonVisibleCheck->setEnabled(false);
    }

    updateVisualization();
}

/**
 * @brief Colorize mesh depending on which radio button you have chosen
 */
void FourAxisFabricationManager::updateVisualization() {
    if (ui->meshRadio->isChecked()) {
        colorizeMesh();
    }
    else if (ui->extremesRadio->isChecked()) {
        colorizeExtremes();
    }
    else if (ui->visibilityRadio->isChecked()) {
        colorizeVisibility();
    }
    else if (ui->associationRadio->isChecked()) {
        colorizeAssociation();
    }
    else if (ui->resultsRadio->isChecked()) {
        showResults();
    }


    //Update canvas
    mainWindow.canvas.update();

    updateUI();
}

/**
 * @brief Colorize mesh to the default color
 */
void FourAxisFabricationManager::colorizeMesh() {
    //Set default color
    drawableSmoothedMesh.setFaceColor(cg3::Color(128,128,128));
    drawableRestoredMesh.setFaceColor(cg3::Color(128,128,128));
    drawableMinComponent.setFaceColor(cg3::Color(128,128,128));
    drawableMaxComponent.setFaceColor(cg3::Color(128,128,128));
    drawableFourAxisComponent.setFaceColor(cg3::Color(128,128,128));

    ui->descriptionLabel->setText(""); //Empty description text
}


/**
 * @brief Colorize the min and max extremes
 */
void FourAxisFabricationManager::colorizeExtremes() {
    //Set default color
    drawableSmoothedMesh.setFaceColor(cg3::Color(128,128,128));
    drawableRestoredMesh.setFaceColor(cg3::Color(128,128,128));

    unsigned int sliderValue = (unsigned int) ui->visualizationSlider->value();

    //Color the min and max extremes
    if (sliderValue == 0 || sliderValue == 1) {
        for (unsigned int i : data.minExtremes){
            drawableSmoothedMesh.setFaceColor(cg3::Color(0,0,255), i);
        }
    }
    if (sliderValue == 0 || sliderValue == 2) {
        for (unsigned int i : data.maxExtremes){
            drawableSmoothedMesh.setFaceColor(cg3::Color(255,0,0), i);
        }
    }

    //Description
    cg3::Vec3 xAxis(1,0,0);
    cg3::Vec3 xAxisOpposite(-1,0,0);

    std::stringstream ss;
    switch (sliderValue) {
    case 1:
        ss << "Min: " << xAxisOpposite;
        break;
    case 2:
        ss << "Max: " << xAxis;
        break;
    default:
        ss << "All";
    }

    std::string description = ss.str();
    ui->descriptionLabel->setText(QString::fromStdString(description));
}

/**
 * @brief Colorize face visibility from directions
 */
void FourAxisFabricationManager::colorizeVisibility() {
    unsigned int sliderValue = (unsigned int) ui->visualizationSlider->value();
    unsigned int showNonVisible = (unsigned int) ui->showNonVisibleCheck->isChecked();

    //Set default color
    drawableSmoothedMesh.setFaceColor(cg3::Color(128,128,128));
    drawableRestoredMesh.setFaceColor(cg3::Color(128,128,128));


    //Information on visibility
    if (sliderValue == 0) {
        showCurrentStatusDescription();
    }
    else {
        //Subdivisions for colors
        int subd = 255 / (data.directions.size()-1);

        //Get index of the current direction
        int chosenDirectionIndex = sliderValue - 1;

        //Set color for the current direction
        cg3::Color color;
        color.setHsv(subd * chosenDirectionIndex, 255, 255);

        //Color the faces visible from that direction
        for (unsigned int j = 0; j < data.visibility.sizeY(); j++) {
            if (data.visibility(chosenDirectionIndex, j) == 1) {
                //Set the color
                drawableSmoothedMesh.setFaceColor(color, j);
            }
        }

        std::stringstream ss;

        //Description
        ss << "Direction " << data.directions[chosenDirectionIndex];

        //Update description label
        std::string description = ss.str();
        ui->descriptionLabel->setText(QString::fromStdString(description));
    }

    //Show non-visible faces
    if (showNonVisible) {
        for (unsigned int faceId : data.nonVisibleFaces) {
            drawableSmoothedMesh.setFaceColor(cg3::Color(0,0,0), faceId);
        }
    }

}



/**
 * @brief Colorize association
 */
void FourAxisFabricationManager::colorizeAssociation() {
    //Coloring drawable mesh
    colorizeAssociation(drawableSmoothedMesh, data.association, data.targetDirections, data.associationNonVisibleFaces);

    //Coloring restored mesh
    if (data.areFrequenciesRestored) {
        colorizeAssociation(drawableRestoredMesh, data.restoredMeshAssociation, data.targetDirections, data.restoredMeshNonVisibleFaces);
    }

    //Coloring cut components
    if (data.areComponentsCut) {
        colorizeAssociation(drawableFourAxisComponent, data.fourAxisAssociation, data.targetDirections, data.fourAxisNonVisibleFaces);
    }

    //Get UI data
    unsigned int sliderValue = (unsigned int) ui->visualizationSlider->value();

    //Description
    if (sliderValue > 0) {
        //Get index of the current chosen direction
        unsigned int chosenDirectionIndex = data.targetDirections[sliderValue - 1];

        std::stringstream ss;

        ss << "Direction " << data.directions[chosenDirectionIndex];

        //Update description label
        std::string description = ss.str();
        ui->descriptionLabel->setText(QString::fromStdString(description));
    }
    else {
        showCurrentStatusDescription();
    }

}


/**
 * @brief Colorize association for a given mesh
 * @param drawableMesh Drawable mesh to be colorized
 * @param association Association of the target directions
 * @param targetDirections Target directions
 */
void FourAxisFabricationManager::colorizeAssociation(
        cg3::DrawableEigenMesh& drawableMesh,
        const std::vector<int>& association,
        const std::vector<unsigned int>& targetDirections,
        const std::vector<unsigned int>& nonVisibleFaces)
{
    unsigned int sliderValue = (unsigned int) ui->visualizationSlider->value();
    unsigned int showNonVisible = (unsigned int) ui->showNonVisibleCheck->isChecked();

    //Variables for colors
    cg3::Color color;
    int subd = 255 / (data.targetDirections.size());

    //Set the color
    drawableMesh.setFaceColor(cg3::Color(128,128,128));

    //For each face of the drawable smoothed mesh
    for (unsigned int faceId = 0; faceId < drawableMesh.numberFaces(); faceId++) {
        //Get direction index associated to the current face
        int associatedDirectionIndex = association[faceId];

        //If it has an associated fabrication direction
        if (associatedDirectionIndex >= 0) {
            //Find position in target directions to set the color
            std::vector<unsigned int>::const_iterator it =
                    std::find(targetDirections.begin(), targetDirections.end(), associatedDirectionIndex);

            int positionInTargetDirections = std::distance(targetDirections.begin(), it);

            color.setHsv(subd * positionInTargetDirections, 255, 255);

            if (sliderValue == 0 ||
                    data.targetDirections[sliderValue-1] == (unsigned int) associatedDirectionIndex)
            {
                //Set the color
                drawableMesh.setFaceColor(color, faceId);
            }
        }
    }

    //Show non-visible faces
    if (showNonVisible) {
        for (unsigned int faceId : nonVisibleFaces) {
            drawableMesh.setFaceColor(cg3::Color(0,0,0), faceId);
        }
    }
}



/**
 * @brief Colorize face visibility from the target directions
 */
void FourAxisFabricationManager::showResults() {
    unsigned int sliderValue = (unsigned int) ui->visualizationSlider->value();

    //Get index of the current direction
    unsigned int resultLabel = data.resultsAssociation[sliderValue];

    std::stringstream ss;

    //Description
    ss << "Direction " << data.directions[resultLabel];

    //Update description label
    std::string description = ss.str();
    ui->descriptionLabel->setText(QString::fromStdString(description));

}

/**
 * @brief Show description of the current status
 */
void FourAxisFabricationManager::showCurrentStatusDescription()
{
    if (data.isAssociationComputed) {
        std::stringstream ss;

        //Description
        ss << "Target directions: " << data.targetDirections.size() << " directions.";

        //Non-visible data
        ss << " Non-visible: " << data.nonVisibleFaces.size() << ".";
        if (data.isAssociationOptimized)
            ss << " Ass: " << data.associationNonVisibleFaces.size() << ".";
        if (data.isVisibilityRecheckedAfterRestore)
            ss << " Freq: "<<  data.restoredMeshNonVisibleFaces.size() << ".";

        //Update description label
        std::string description = ss.str();
        ui->descriptionLabel->setText(QString::fromStdString(description));
    }
}


/* ----- UI SLOTS MESH ------ */

void FourAxisFabricationManager::on_loadMeshButton_clicked()
{
    if (!data.isMeshLoaded) {
        std::string meshFile;
        std::string smoothedFile;

        //Get loading dialog
        meshFile = loaderSaverObj.loadDialog("Load mesh");

        if (meshFile != "") {
            data.isMeshLoaded = data.originalMesh.loadFromObj(meshFile);

            //If the mesh has been successfully loaded
            if (data.isMeshLoaded){
                std::string rawname, ext;
                cg3::separateExtensionFromFilename(meshFile, rawname, ext);

                //Find smoothed mesh in the path
                smoothedFile = rawname + "_smooth" + ext;
                data.isMeshLoaded = data.originalSmoothedMesh.loadFromObj(smoothedFile);

                //If a smoothed mesh has not been found
                if (!data.isMeshLoaded){
                    //Get loading dialog
                    smoothedFile = loaderSaverObj.loadDialog("Load smoothed mesh");
                    if (smoothedFile != ""){
                        data.isMeshLoaded = data.originalSmoothedMesh.loadFromObj(smoothedFile);
                    }
                }

                //If a smoothed mesh has been found
                if (data.isMeshLoaded) {
                    std::string meshName = meshFile.substr(meshFile.find_last_of("/") + 1);
                    std::string smoothedMeshName = meshFile.substr(meshFile.find_last_of("/") + 1);

                    data.mesh = data.originalMesh;
                    data.smoothedMesh = data.originalSmoothedMesh;

                    addDrawableMeshes();

                    //Visualize mesh
                    ui->meshRadio->setChecked(true);
                    initializeVisualizationSlider();

                    //Update canvas and fit the scene
                    mainWindow.canvas.update();
                    mainWindow.canvas.fitScene();

                    std::cout << std::endl;

                    std::cout << "Original: \"" << meshName << "\" " << "(" << data.mesh.numberFaces() << " F / " << data.mesh.numberVertices() << " V)" << std::endl;
                    std::cout << "Smoothed: \"" << smoothedMeshName << "\" " << "(" << data.originalSmoothedMesh.numberFaces() << " F / " << data.smoothedMesh.numberVertices() << " V)" << std::endl;
                }
                else {
                    clearData();
                }
            }
        }

        updateUI();

        resetCameraDirection();
    }
}

void FourAxisFabricationManager::on_clearMeshButton_clicked()
{
    if (data.isMeshLoaded) {
        //Delete objects from the canvas
        deleteDrawableObjects();

        //Clear four axis fabrication data
        clearData();

        //Update canvas and fit the scene
        mainWindow.canvas.update();
        mainWindow.canvas.fitScene();

        //Visualize mesh
        ui->meshRadio->setChecked(true);
        initializeVisualizationSlider();

        updateUI();
    }

}

void FourAxisFabricationManager::on_reloadMeshButton_clicked()
{
    if (data.isMeshLoaded) {
        //Delete drawable Objects
        deleteDrawableObjects();

        //Saving for reloading
        cg3::EigenMesh originalMeshCopy = data.originalMesh;
        cg3::EigenMesh originalSmoothedMeshCopy = data.originalSmoothedMesh;

        //Clear four axis fabrication data
        clearData();

        //Reload
        data.originalMesh = originalMeshCopy;
        data.originalSmoothedMesh = originalSmoothedMeshCopy;

        data.mesh = originalMeshCopy;
        data.smoothedMesh = originalSmoothedMeshCopy;

        data.isMeshLoaded = true;

        //Add drawable meshes
        addDrawableMeshes();

        //Update canvas and fit the scene
        mainWindow.canvas.update();
        mainWindow.canvas.fitScene();

        //Visualize mesh
        ui->meshRadio->setChecked(true);
        initializeVisualizationSlider();

        updateUI();

        resetCameraDirection();
    }
}

void FourAxisFabricationManager::on_saveResultsButton_clicked() {
    //Get saving dialog
    std::string selectedExtension;
    std::string saveFileName = loaderSaverObj.saveDialog("Save mesh", selectedExtension);
    saveFileName += "." + selectedExtension;

    if (saveFileName != "") {
        std::string rawname, ext;
        cg3::separateExtensionFromFilename(saveFileName, rawname, ext);

        //Save on obj files
        data.mesh.saveOnObj(rawname + "_original.obj");
        data.smoothedMesh.saveOnObj(rawname + "_smoothed.obj");

        if (data.isAssociationComputed) {
            std::ofstream resultFile;
            resultFile.open (rawname + "_directions.txt");
            for (size_t i = 0; i < data.targetDirections.size(); i++) {
                size_t label = data.targetDirections[i];
                resultFile << i << " -> " <<
                              "Label " << label << ", " <<
                              "Direction: " << data.directions[label];

                if (i < data.targetDirections.size()-2) {
                    resultFile << ", " <<
                              "Angle: " << data.angles[label] <<
                              " (" << data.angles[label]/M_PI*180 << "°)";
                }

                resultFile << std::endl;
            }
            resultFile.close();

            if (data.areFrequenciesRestored) {
                data.restoredMesh.saveOnObj(rawname + "_restored.obj");

                if (data.areComponentsCut) {
                    data.minComponent.setVertexColor(128,128,128);
                    data.maxComponent.setVertexColor(128,128,128);
                    data.fourAxisComponent.setVertexColor(128,128,128);

                    if (data.minComponent.numberFaces() > 0) {
                        data.minComponent.saveOnObj(rawname + "_component_min.obj");
                    }
                    if (data.maxComponent.numberFaces() > 0) {
                        data.maxComponent.saveOnObj(rawname + "_component_max.obj");
                    }
                    data.fourAxisComponent.saveOnObj(rawname + "_component_fouraxis.obj");

                    if (data.areResultsExtracted) {
                        for (size_t i = 0; i < data.stocks.size(); i++) {
                            cg3::EigenMesh& mesh = data.stocks[i];
                            mesh.setVertexColor(128,128,128);
                            mesh.saveOnObj(rawname + "_stock_" + std::to_string(i) + ".obj");
                        }

                        for (size_t i = 0; i < data.results.size(); i++) {
                            cg3::EigenMesh& mesh = data.results[i];
                            mesh.setVertexColor(128,128,128);
                            mesh.saveOnObj(rawname + "_result_" + std::to_string(i) + ".obj");
                        }

                        if (data.minResult.numberFaces() > 0) {
                            data.minResult.saveOnObj(rawname + "_result_min.obj");
                        }
                        if (data.maxResult.numberFaces() > 0) {
                            data.maxResult.saveOnObj(rawname + "_result_max.obj");
                        }

                        if (data.minSupport.numberFaces() > 0) {
                            data.minSupport.saveOnObj(rawname + "_support_min.obj");
                        }

                        if (data.maxSupport.numberFaces() > 0) {
                            data.maxSupport.saveOnObj(rawname + "_support_max.obj");
                        }
                    }
                }
            }
        }
    }
}


/* ----- LOAD/SAVE DATA SLOTS ------ */

void FourAxisFabricationManager::on_loadDataButton_clicked()
{
    //Get loading dialog
    std::string dataFile = loaderSaverData.loadDialog("Load data");

    if (dataFile != "") {
        std::ifstream inputStream(dataFile);
        data.deserialize(inputStream);
    }

    if (data.isMeshLoaded) {
        addDrawableMeshes();

        //Visualize mesh
        ui->meshRadio->setChecked(true);
    }

    if (data.areExtremesSelected) {
        //Visualize mesh
        ui->extremesRadio->setChecked(true);
    }

    if (data.isVisibilityChecked) {
        //Visualize mesh
        ui->visibilityRadio->setChecked(true);
    }

    if (data.isAssociationComputed) {
        //Visualize mesh
        ui->associationRadio->setChecked(true);
    }

    if (data.areFrequenciesRestored) {
        addDrawableRestoredMesh();
    }

    if (data.areComponentsCut) {
        addDrawableCutComponents();
    }

    if (data.areResultsExtracted) {
        addDrawableResults();

        //Visualize mesh
        ui->resultsRadio->setChecked(true);
    }

    initializeVisualizationSlider();

    updateUI();

    mainWindow.canvas.update();
    mainWindow.canvas.fitScene();
}

void FourAxisFabricationManager::on_saveDataButton_clicked()
{
    //Get saving dialog
    std::string selectedExtension;
    std::string saveFileName = loaderSaverData.saveDialog("Save data", selectedExtension);

    if (saveFileName != "") {
        std::ofstream outputStream(saveFileName);
        data.serialize(outputStream);
    }
}

/* ----- UI SLOTS FOUR AXIS FABRICATION ------ */


void FourAxisFabricationManager::on_optimalOrientationButton_clicked() {
    //Get optimal mesh orientation
    optimalOrientation();

    //Update canvas and fit the scene
    mainWindow.canvas.update();
    mainWindow.canvas.fitScene();

    //Visualize mesh
    ui->meshRadio->setChecked(true);
    initializeVisualizationSlider();

    updateUI();
}

void FourAxisFabricationManager::on_selectExtremesButton_clicked()
{
    //Get extremes on x-axis to be selected
    selectExtremes();

    //Update canvas and fit the scene
    mainWindow.canvas.update();
    mainWindow.canvas.fitScene();

    //Visualize extremes
    ui->extremesRadio->setChecked(true);
    initializeVisualizationSlider();

    updateUI();
}

void FourAxisFabricationManager::on_checkVisibilityButton_clicked()
{
    //Check visibility by the chosen directions
    checkVisibility();

    //Update canvas and fit the scene
    mainWindow.canvas.update();
    mainWindow.canvas.fitScene();

    //Visualize visibility
    ui->visibilityRadio->setChecked(true);
    initializeVisualizationSlider();

    updateUI();
}

void FourAxisFabricationManager::on_getAssociationButton_clicked()
{
    //Check visibility by the chosen directions
    getAssociation();

    //Update canvas and fit the scene
    mainWindow.canvas.update();
    mainWindow.canvas.fitScene();

    //Visualize association
    ui->associationRadio->setChecked(true);
    initializeVisualizationSlider();

    updateUI();
}

void FourAxisFabricationManager::on_optimizationButton_clicked()
{
    //Optimization
    optimizeAssociation();

    //Update canvas and fit the scene
    mainWindow.canvas.update();
    mainWindow.canvas.fitScene();

    //Visualize association
    ui->associationRadio->setChecked(true);
    initializeVisualizationSlider();

    updateUI();
}

void FourAxisFabricationManager::on_restoreFrequenciesButton_clicked() {
    //Restore frequencies
    restoreFrequencies();

    //Visualize association
    ui->associationRadio->setChecked(true);
    initializeVisualizationSlider();

    //Update canvas and fit the scene
    mainWindow.canvas.update();
    mainWindow.canvas.fitScene();

    updateUI();
}

void FourAxisFabricationManager::on_recheckVisibilityButton_clicked()
{
    //Recheck visibility
    recheckVisibilityAfterRestore();

    //Visualize association
    ui->associationRadio->setChecked(true);
    initializeVisualizationSlider();

    //Update canvas and fit the scene
    mainWindow.canvas.update();
    mainWindow.canvas.fitScene();

    updateUI();
}

void FourAxisFabricationManager::on_cutComponentsButton_clicked() {
    //Cut components
    cutComponents();

    //Update canvas and fit the scene
    mainWindow.canvas.update();
    mainWindow.canvas.fitScene();

    //Visualize association
    ui->associationRadio->setChecked(true);
    initializeVisualizationSlider();

    updateUI();
}


void FourAxisFabricationManager::on_extractResultsButton_clicked() {
    //Extract results
    extractResults();

    //Update canvas and fit the scene
    mainWindow.canvas.update();
    mainWindow.canvas.fitScene();

    //Visualize association
    ui->resultsRadio->setChecked(true);
    initializeVisualizationSlider();

    updateUI();
}

/* ----- UI SLOTS TRANSFORMATIONS ------ */


void FourAxisFabricationManager::on_centerOnOriginButton_clicked() {
    if (data.isMeshLoaded){
        //Translation of the mesh
        data.mesh.translate(-data.smoothedMesh.boundingBox().center());
        data.smoothedMesh.translate(-data.smoothedMesh.boundingBox().center());

        //Update canvas and fit the scene
        mainWindow.canvas.update();
        mainWindow.canvas.fitScene();
    }
}

void FourAxisFabricationManager::on_plusXButton_clicked() {
    if (data.isMeshLoaded){
        //Translation of the mesh
        data.mesh.translate(cg3::Pointd(ui->stepSpinBox->value(), 0, 0));
        data.smoothedMesh.translate(cg3::Pointd(ui->stepSpinBox->value(), 0, 0));

        //Update canvas and fit the scene
        mainWindow.canvas.update();
        mainWindow.canvas.fitScene();
    }
}

void FourAxisFabricationManager::on_minusXButton_clicked() {
    if (data.isMeshLoaded){
        //Translation of the mesh
        data.mesh.translate(cg3::Pointd(-ui->stepSpinBox->value(), 0, 0));
        data.smoothedMesh.translate(cg3::Pointd(-ui->stepSpinBox->value(), 0, 0));

        //Update canvas and fit the scene
        mainWindow.canvas.update();
        mainWindow.canvas.fitScene();
    }
}

void FourAxisFabricationManager::on_plusYButton_clicked() {
    if (data.isMeshLoaded){
        //Translation of the mesh
        data.mesh.translate(cg3::Pointd(0, ui->stepSpinBox->value(), 0));
        data.smoothedMesh.translate(cg3::Pointd(0, ui->stepSpinBox->value(), 0));

        //Update canvas and fit the scene
        mainWindow.canvas.update();
        mainWindow.canvas.fitScene();
    }
}

void FourAxisFabricationManager::on_minusYButton_clicked() {
    if (data.isMeshLoaded){
        //Translation of the mesh
        data.mesh.translate(cg3::Pointd(0, -ui->stepSpinBox->value(), 0));
        data.smoothedMesh.translate(cg3::Pointd(0, -ui->stepSpinBox->value(), 0));

        //Update canvas and fit the scene
        mainWindow.canvas.update();
        mainWindow.canvas.fitScene();
    }
}

void FourAxisFabricationManager::on_plusZButton_clicked() {
    if (data.isMeshLoaded){
        //Translation of the mesh
        data.mesh.translate(cg3::Pointd(0, 0, ui->stepSpinBox->value()));
        data.smoothedMesh.translate(cg3::Pointd(0, 0, ui->stepSpinBox->value()));

        //Update canvas and fit the scene
        mainWindow.canvas.update();
        mainWindow.canvas.fitScene();
    }
}

void FourAxisFabricationManager::on_minusZButton_clicked() {
    if (data.isMeshLoaded){
        //Translation of the mesh
        data.mesh.translate(cg3::Pointd(0, 0, -ui->stepSpinBox->value()));
        data.smoothedMesh.translate(cg3::Pointd(0, 0, -ui->stepSpinBox->value()));

        //Update canvas and fit the scene
        mainWindow.canvas.update();
        mainWindow.canvas.fitScene();
    }
}



void FourAxisFabricationManager::on_rotateButton_clicked() {
    if (data.isMeshLoaded){
        //Rotation of the mesh
        cg3::Vec3 axis(ui->axisXSpinBox->value(), ui->axisYSpinBox->value(), ui->axisZSpinBox->value());
        double angle = ui->angleSpinBox->value() * M_PI/180;

        Eigen::Matrix3d m;
        cg3::rotationMatrix(axis, angle, m);

        data.mesh.rotate(m);
        data.smoothedMesh.rotate(m);

        //Update canvas and fit the scene
        mainWindow.canvas.update();
        mainWindow.canvas.fitScene();
    }
}




void FourAxisFabricationManager::on_scaleButton_clicked() {
    if (data.isMeshLoaded){
        //Scale the mesh
        cg3::Vec3 scaleFactor(ui->scaleXSpinBox->value(), ui->scaleYSpinBox->value(), ui->scaleZSpinBox->value());

        data.mesh.scale(scaleFactor);
        data.smoothedMesh.scale(scaleFactor);

        //Update canvas and fit the scene
        mainWindow.canvas.update();
        mainWindow.canvas.fitScene();
    }
}

void FourAxisFabricationManager::on_inverseScaleButton_clicked() {
    if (data.isMeshLoaded){
        //Scale the mesh
        cg3::Vec3 scaleFactor(1.0/ui->scaleXSpinBox->value(), 1.0/ui->scaleYSpinBox->value(), 1.0/ui->scaleZSpinBox->value());

        data.mesh.scale(scaleFactor);
        data.smoothedMesh.scale(scaleFactor);

        //Update canvas and fit the scene
        mainWindow.canvas.update();
        mainWindow.canvas.fitScene();
    }
}




/* ----- UI SLOTS VISUALIZATION ------ */

void FourAxisFabricationManager::on_meshRadio_clicked() {
    initializeVisualizationSlider();
}

void FourAxisFabricationManager::on_extremesRadio_clicked() {
    initializeVisualizationSlider();
}

void FourAxisFabricationManager::on_visibilityRadio_clicked() {
    initializeVisualizationSlider();
}

void FourAxisFabricationManager::on_associationRadio_clicked() {
    initializeVisualizationSlider();
}

void FourAxisFabricationManager::on_resultsRadio_clicked() {
    initializeVisualizationSlider();
}

void FourAxisFabricationManager::on_showNonVisibleCheck_clicked()
{
    initializeVisualizationSlider();
}
void FourAxisFabricationManager::on_resetCameraButton_clicked() {
    //Reset camera direction
    resetCameraDirection();
}

void FourAxisFabricationManager::on_visualizationSlider_valueChanged(int value) {
    CG3_SUPPRESS_WARNING(value);
    updateVisualization();

    unsigned int sliderValue = (unsigned int) ui->visualizationSlider->value();

    if (ui->moveCameraCheckBox->isChecked()) {
        if (ui->extremesRadio->isChecked()) {
            cg3::Vec3 xAxis(1,0,0);
            if (sliderValue == 1) { //Min
                setCameraDirection(xAxis);
            }
            else if (sliderValue == 2) { //Max
                setCameraDirection(-xAxis);
            }
            else if (sliderValue == 0) {
                resetCameraDirection();
            }
        }
        else if (ui->visibilityRadio->isChecked()) {
            if (sliderValue > 0) {
                cg3::Vec3 currentDirection = data.directions[sliderValue - 1];
                setCameraDirection(-currentDirection);
            }
            else {
                resetCameraDirection();
            }
        }
        else if (ui->associationRadio->isChecked()) {
            if (sliderValue > 0) {
                unsigned int currentIndex = data.targetDirections[sliderValue - 1];
                cg3::Vec3 currentDirection = data.directions[currentIndex];
                setCameraDirection(-currentDirection);
            }
            else {
                resetCameraDirection();
            }
        }
        else if (ui->resultsRadio->isChecked()) {
            if (ui->extractResultsRotateCheckBox->isChecked()) {
                cg3::Vec3 zAxis(0,0,1);
                setCameraDirection(-zAxis);
            }
            else {
                unsigned int currentIndex = data.targetDirections[sliderValue];
                cg3::Vec3 currentDirection = data.directions[currentIndex];
                setCameraDirection(-currentDirection);
            }

            for (cg3::DrawableEigenMesh& res : drawableResults) {
                mainWindow.setDrawableObjectVisibility(&res, false);
            }
            mainWindow.setDrawableObjectVisibility(&drawableResults[sliderValue], true);
        }
    }
}
