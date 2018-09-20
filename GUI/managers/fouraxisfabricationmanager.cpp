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

    mainWindow.canvas.setOrthographicCamera();

    clearData();

    updateUI();
}

/**
 * @brief Update UI depending on the current state
 */
void FourAxisFabricationManager::updateUI() {
    // ----- Mesh loading -----
    ui->loadMeshButton->setEnabled(!isMeshLoaded);
    ui->clearMeshButton->setEnabled(isMeshLoaded);
    ui->reloadMeshButton->setEnabled(isMeshLoaded);
    ui->saveResultsButton->setEnabled(isMeshLoaded);


    // ----- Four axis fabrication -----
    ui->fourAxisFabricationGroup->setEnabled(isMeshLoaded);

    //Optimal orientation
    ui->optimalOrientationButton->setEnabled(!isMeshOriented);
    ui->optimalOrientationOrientationsLabel->setEnabled(!isMeshOriented);
    ui->optimalOrientationOrientationsSpinBox->setEnabled(!isMeshOriented);
    ui->optimalOrientationDeterministicCheckBox->setEnabled(!isMeshOriented);

    //Select extremes
    ui->selectExtremesButton->setEnabled(!areExtremesSelected);
    ui->selectExtremesHeightfieldAngleLabel->setEnabled(!areExtremesSelected);
    ui->selectExtremesHeightfieldAngleSpinBox->setEnabled(!areExtremesSelected);

    //Check visibility
    ui->checkVisibilityButton->setEnabled(!isVisibilityChecked);
    ui->checkVisibilityDirectionsLabel->setEnabled(!isVisibilityChecked);
    ui->checkVisibilityDirectionsSpinBox->setEnabled(!isVisibilityChecked);
    ui->checkVisibilityResolutionLabel->setEnabled(!isVisibilityChecked);
    ui->checkVisibilityResolutionSpinBox->setEnabled(!isVisibilityChecked);
    ui->checkVisibilityXDirectionsCheckBox->setEnabled(!isVisibilityChecked);

    //Get the target directions
    ui->targetDirectionsButton->setEnabled(!areTargetDirectionsFound);
    ui->targetDirectionsSetCoverageCheckBox->setEnabled(!areTargetDirectionsFound);

    //Get association
    ui->getAssociationButton->setEnabled(!isAssociationComputed);
    ui->getAssociationDataSigmaSpinBox->setEnabled(!isAssociationComputed);
    ui->getAssociationDataSigmaLabel->setEnabled(!isAssociationComputed);
    ui->getAssociationFreeCostAngleSpinBox->setEnabled(!isAssociationComputed);
    ui->getAssociationFreeCostAngleLabel->setEnabled(!isAssociationComputed);
    ui->getAssociationMaxLabelAngleSpinBox->setEnabled(!isAssociationComputed);
    ui->getAssociationMaxLabelAngleLabel->setEnabled(!isAssociationComputed);
    ui->getAssociationCompactnessLabel->setEnabled(!isAssociationComputed);
    ui->getAssociationCompactnessSpinBox->setEnabled(!isAssociationComputed);
    ui->getAssociationFixExtremesCheckBox->setEnabled(!isAssociationComputed);

    //Optimization
    ui->optimizationButton->setEnabled(!isAssociationOptimized);
    ui->optimizationRelaxHolesCheckBox->setEnabled(!isAssociationOptimized);
    ui->optimizationLoseHolesCheckBox->setEnabled(!isAssociationOptimized);
    ui->optimizationMinChartAreaLabel->setEnabled(!isAssociationOptimized);
    ui->optimizationMinChartAreaSpinBox->setEnabled(!isAssociationOptimized);

    //Restore frequencies
    ui->restoreFrequenciesButton->setEnabled(!areFrequenciesRestored);
    ui->restoreFrequenciesIterationsLabel->setEnabled(!areFrequenciesRestored);
    ui->restoreFrequenciesIterationsSpinBox->setEnabled(!areFrequenciesRestored);

    //Recheck visibility after restore
    ui->recheckVisibilityReassignNonVisibleCheckBox->setEnabled(!isVisibilityRecheckedAfterRestore);
    ui->recheckVisibilityButton->setEnabled(!isVisibilityRecheckedAfterRestore);

    //Cut components
    ui->cutComponentsButton->setEnabled(!areComponentsCut);
    ui->cutComponentsCheckBox->setEnabled(!areComponentsCut);

    //Extract results
    ui->extractResultsButton->setEnabled(!areResultsExtracted);
    ui->extractResultsStockLengthLabel->setEnabled(!areResultsExtracted);
    ui->extractResultsStockLengthSpinBox->setEnabled(!areResultsExtracted);
    ui->extractResultsStockDiameterLabel->setEnabled(!areResultsExtracted);
    ui->extractResultsStockDiameterSpinBox->setEnabled(!areResultsExtracted);
    ui->extractResultsModelLengthLabel->setEnabled(!areResultsExtracted);
    ui->extractResultsModelLengthSpinBox->setEnabled(!areResultsExtracted);
    ui->extractResultsMillableAngleLabel->setEnabled(!areResultsExtracted);
    ui->extractResultsMillableAngleSpinBox->setEnabled(!areResultsExtracted);
    ui->extractResultsSupportHeightSpinBox->setEnabled(!areResultsExtracted);
    ui->extractResultsSupportHeightLabel->setEnabled(!areResultsExtracted);
    ui->extractResultsRotateCheckBox->setEnabled(!areResultsExtracted);



    // ----- Visualization -----
    ui->visualizationGroup->setEnabled(isMeshLoaded);

    //Radio
    ui->meshRadio->setEnabled(isMeshLoaded);
    ui->extremesRadio->setEnabled(areExtremesSelected);
    ui->visibilityRadio->setEnabled(isVisibilityChecked);
    ui->targetDirectionsRadio->setEnabled(areTargetDirectionsFound);
    ui->associationRadio->setEnabled(isAssociationComputed);


    // ----- Transformations -----
    ui->transformationGroup->setEnabled(isMeshLoaded);
}

/**
 * @brief Clear data of four axis fabrication
 */
void FourAxisFabricationManager::clearData() {
    isMeshLoaded = false;
    isMeshOriented = false;   
    areExtremesSelected = false;
    isVisibilityChecked = false;
    areTargetDirectionsFound = false;
    isAssociationComputed = false;
    isAssociationOptimized = false;
    areFrequenciesRestored = false;
    isVisibilityRecheckedAfterRestore = false;
    areComponentsCut = false;
    areResultsExtracted = false;

    data.clear();

    originalMesh.clear();
    smoothedMesh.clear();
}




/* ----- COMPUTING METHODS ------ */

/**
 * @brief Compute optimal orientation
 */
void FourAxisFabricationManager::optimalOrientation() {
    if (!isMeshOriented) {
        std::cout << std::endl << "#######################################################################" << std::endl << std::endl;

        //Get UI data
        unsigned int nOrientations = (unsigned int) ui->optimalOrientationOrientationsSpinBox->value();
        bool deterministic = ui->optimalOrientationDeterministicCheckBox->isChecked();

        cg3::Timer t("Optimal orientation");

        //Get optimal mesh orientation
        FourAxisFabrication::rotateToOptimalOrientation(
                    originalMesh,
                    smoothedMesh,
                    nOrientations,
                    deterministic);

        t.stopAndPrint();

        isMeshOriented = true;

        updateDrawableMeshes();
    }
}

/**
 * @brief Select extremes
 */
void FourAxisFabricationManager::selectExtremes() {
    if (!areExtremesSelected) {
        optimalOrientation();

        //Get UI data
        double heightfieldAngle = ui->selectExtremesHeightfieldAngleSpinBox->value() / 180.0 * M_PI;

        cg3::Timer t("Select extremes");

        //Get extremes on x-axis to be selected
        FourAxisFabrication::selectExtremesOnXAxis(smoothedMesh, heightfieldAngle, data);

        t.stopAndPrint();


        areExtremesSelected = true;
    }
}

/**
 * @brief Check visibility from various directions
 */
void FourAxisFabricationManager::checkVisibility() {
    if (!isVisibilityChecked) {
        selectExtremes();

        //Get UI data
        double heightfieldAngle = ui->selectExtremesHeightfieldAngleSpinBox->value() / 180.0 * M_PI;
        unsigned int nDirections = (unsigned int) ui->checkVisibilityDirectionsSpinBox->value();
        unsigned int resolution = (unsigned int) ui->checkVisibilityResolutionSpinBox->value();
        bool includeXDirections = ui->checkVisibilityXDirectionsCheckBox->isChecked();

        cg3::Timer t("Visibility check");

        //Visibility check
        FourAxisFabrication::getVisibility(
                    smoothedMesh,
                    nDirections,
                    resolution,
                    heightfieldAngle,
                    includeXDirections,
                    data);

        t.stopAndPrint();

        isVisibilityChecked = true;

        std::cout << "Non-visible triangles: " << data.nonVisibleFaces.size() << std::endl;
    }
}

/**
 * @brief Get the target directions
 */
void FourAxisFabricationManager::getTargetDirections() {
    if (!areTargetDirectionsFound) {
        checkVisibility();

        //Get UI data
        bool setCoverageFlag = ui->targetDirectionsSetCoverageCheckBox->isChecked();

        cg3::Timer t("Get target directions");

        //Get the target fabrication directions
        FourAxisFabrication::getTargetDirections(
                    setCoverageFlag,
                    data);

        t.stopAndPrint();

        areTargetDirectionsFound = true;
    }
}

/**
 * @brief Get association
 */
void FourAxisFabricationManager::getAssociation() {
    if (!isAssociationComputed) {
        getTargetDirections();

        //Get UI data
        double freeCostAngle = ui->getAssociationFreeCostAngleSpinBox->value() / 180.0 * M_PI;
        double dataSigma = ui->getAssociationDataSigmaSpinBox->value();
        double maxLabelAngle = ui->getAssociationMaxLabelAngleSpinBox->value() / 180.0 * M_PI;
        double compactness = ui->getAssociationCompactnessSpinBox->value();
        bool fixExtremes = ui->getAssociationFixExtremesCheckBox->isChecked();

        cg3::Timer t("Get association");

        //Get association
        FourAxisFabrication::getAssociation(
                    smoothedMesh,
                    freeCostAngle,
                    dataSigma,
                    maxLabelAngle,
                    compactness,
                    fixExtremes,
                    data);

        t.stopAndPrint();

        isAssociationComputed = true;
    }
}


/**
 * @brief Get optimized association
 */
void FourAxisFabricationManager::optimizeAssociation() {
    if (!isAssociationOptimized) {
        getAssociation();

        //Get UI data
        bool relaxHoles = ui->optimizationRelaxHolesCheckBox->isChecked();
        bool loseHoles = ui->optimizationLoseHolesCheckBox->isChecked();
        double minChartArea = ui->optimizationMinChartAreaSpinBox->value();

        cg3::Timer t("Optimize association");

        //Execute optimization
        FourAxisFabrication::optimization(
                    smoothedMesh,
                    relaxHoles,
                    loseHoles,
                    minChartArea,
                    data);

        t.stopAndPrint();

        isAssociationOptimized = true;

        std::cout << "Non-visible triangles after association: " << data.associationNonVisibleFaces.size() << std::endl;
    }
}


/**
 * @brief Restore frequencies
 */
void FourAxisFabricationManager::restoreFrequencies() {
    if (!areFrequenciesRestored) {
        optimizeAssociation();

        //Get UI data
        double heightfieldAngle = ui->selectExtremesHeightfieldAngleSpinBox->value() / 180.0 * M_PI;
        unsigned int nIterations = (unsigned int) ui->restoreFrequenciesIterationsSpinBox->value();

        double haussDistance = cg3::libigl::hausdorffDistance(originalMesh, smoothedMesh);
        cg3::BoundingBox originalMeshBB = originalMesh.boundingBox();
        double haussDistanceBB = haussDistance/originalMeshBB.diag();

        std::cout << "Smoothed -> Haussdorff distance: " << haussDistance << " (w.r.t. bounding box: " << haussDistanceBB << ")" << std::endl;

        cg3::Timer t("Restore frequencies");

        //Restore frequencies
        FourAxisFabrication::restoreFrequencies(nIterations, heightfieldAngle, originalMesh, smoothedMesh, data);

        t.stopAndPrint();

        haussDistance = cg3::libigl::hausdorffDistance(originalMesh, data.restoredMesh);
        originalMeshBB = originalMesh.boundingBox();
        haussDistanceBB = haussDistance/originalMeshBB.diag();

        std::cout << "Restored -> Haussdorff distance: " << haussDistance << " (w.r.t. bounding box: " << haussDistanceBB << ")" << std::endl;

        areFrequenciesRestored = true;

        addDrawableRestoredMesh();
    }
}


/**
 * @brief Recheck visibility after frequencies are restored
 */
void FourAxisFabricationManager::recheckVisibilityAfterRestore() {
    if (!isVisibilityRecheckedAfterRestore) {
        restoreFrequencies();

        //Get UI data
        double heightfieldAngle = ui->selectExtremesHeightfieldAngleSpinBox->value() / 180.0 * M_PI;
        unsigned int resolution = (unsigned int) ui->checkVisibilityResolutionSpinBox->value();
        bool includeXDirections = ui->checkVisibilityXDirectionsCheckBox->isChecked();
        bool reassign = ui->recheckVisibilityReassignNonVisibleCheckBox->isChecked();


        cg3::Timer tCheck("Recheck visibility after frequencies have been restored");

        //Check if it is a valid association
        FourAxisFabrication::recheckVisibilityAfterRestore(resolution, heightfieldAngle, includeXDirections, reassign, data);

        tCheck.stopAndPrint();

        std::cout << "Non-visible triangles after recheck: " << data.restoredMeshNonVisibleFaces.size() << std::endl;

        isVisibilityRecheckedAfterRestore = true;

        updateDrawableRestoredMesh();
    }
}



/**
 * @brief Cut components
 */
void FourAxisFabricationManager::cutComponents() {
    if (!areComponentsCut) {
        recheckVisibilityAfterRestore();

        //Get UI data
        bool cutComponents = ui->cutComponentsCheckBox->isChecked();



        cg3::Timer t("Cut components");

        //Cut components
        FourAxisFabrication::cutComponents(data, cutComponents);

        t.stopAndPrint();

        areComponentsCut = true;

        addDrawableCutComponents();
    }
}

/**
 * @brief Extract results
 */
void FourAxisFabricationManager::extractResults() {
    if (!areResultsExtracted) {
        cutComponents();

        //Get UI data
        double modelLength = ui->extractResultsModelLengthSpinBox->value();
        double stockLength = ui->extractResultsStockLengthSpinBox->value();
        double stockDiameter = ui->extractResultsStockDiameterSpinBox->value();
        double supportHeight = ui->extractResultsSupportHeightSpinBox->value();
        double millableAngle = ui->extractResultsMillableAngleSpinBox->value() / 180.0 * M_PI;
        bool rotateSurfaces = ui->extractResultsRotateCheckBox->isChecked();

        cg3::Timer t("Extract results");

        //Extract results
        FourAxisFabrication::extractResults(data, modelLength, stockLength, stockDiameter, millableAngle, supportHeight, rotateSurfaces);

        t.stopAndPrint();

        areResultsExtracted = true;

        addDrawableResults();
    }
}




/* ----- VISUALIZATION METHODS ------ */

/**
 * @brief Add drawable meshes
 */
void FourAxisFabricationManager::addDrawableMeshes(const std::string& meshName) {
    //Add drawable meshes to the canvas
    drawableOriginalMesh = cg3::DrawableEigenMesh(originalMesh);
    drawableOriginalMesh.setFlatShading();
    drawableSmoothedMesh = cg3::DrawableEigenMesh(smoothedMesh);
    drawableSmoothedMesh.setFlatShading();

    mainWindow.pushDrawableObject(&drawableOriginalMesh, meshName);
    mainWindow.pushDrawableObject(&drawableSmoothedMesh, "Smoothed mesh");
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
    mainWindow.setDrawableObjectVisibility(&drawableFourAxisComponent, true);
    mainWindow.setDrawableObjectVisibility(&drawableMinComponent, false);
    mainWindow.setDrawableObjectVisibility(&drawableMaxComponent, false);

    //Draw surfaces (hidden by default)
    drawableSurfacesContainer.clear();
    drawableSurfaces.resize(data.surfaces.size());
    for (size_t i = 0; i < data.surfaces.size(); i++) {
        drawableSurfaces[i] = cg3::DrawableEigenMesh(data.surfaces[i]);
        drawableSurfaces[i].setFlatShading();
        drawableSurfacesContainer.pushBack(&drawableSurfaces[i], "Surface " + std::to_string(i), (i == 0 ? true : false));
    }
    mainWindow.pushDrawableObject(&drawableSurfacesContainer, "Surfaces", false);
    if (data.minSurface.numberFaces() > 0) {
        drawableMinSurface = cg3::DrawableEigenMesh(data.minSurface);
        drawableMinSurface.setFlatShading();
        mainWindow.pushDrawableObject(&drawableMinSurface, "Min surface", false);
    }
    if (data.maxSurface.numberFaces() > 0) {
        drawableMaxSurface = cg3::DrawableEigenMesh(data.maxSurface);
        drawableMaxSurface.setFlatShading();
        mainWindow.pushDrawableObject(&drawableMaxSurface, "Max surface", false);
    }

    //Add stock (hidden by default)
    drawableStocksContainer.clear();
    drawableStocks.resize(data.stocks.size());
    for (size_t i = 0; i < data.stocks.size(); i++) {
        drawableStocks[i] = cg3::DrawableEigenMesh(data.stocks[i]);
        drawableStocks[i].setFlatShading();
        drawableStocksContainer.pushBack(&drawableStocks[i], "Stock " + std::to_string(i), (i == 0 ? true : false));
    }
    mainWindow.pushDrawableObject(&drawableStocksContainer, "Stocks", false);

    //Draw results
    drawableResultsContainer.clear();
    drawableResults.resize(data.results.size());
    for (size_t i = 0; i < data.results.size(); i++) {
        drawableResults[i] = cg3::DrawableEigenMesh(data.results[i]);
        drawableResults[i].setFlatShading();
        drawableResultsContainer.pushBack(&drawableResults[i], "Result " + std::to_string(i), (i == 0 ? true : false));
    }
    mainWindow.pushDrawableObject(&drawableResultsContainer, "Results", false);
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

    drawableOriginalMesh = cg3::DrawableEigenMesh(originalMesh);
    drawableSmoothedMesh = cg3::DrawableEigenMesh(smoothedMesh);

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

    mainWindow.setDrawableObjectVisibility(&drawableRestoredMesh, restoredVisibility);
}

/**
 * @brief Delte all drawable objects from the canvas
 */
void FourAxisFabricationManager::deleteDrawableObjects() {
    if (isMeshLoaded) {
        //Delete meshes
        mainWindow.deleteDrawableObject(&drawableOriginalMesh);
        mainWindow.deleteDrawableObject(&drawableSmoothedMesh);

        drawableOriginalMesh.clear();
        drawableSmoothedMesh.clear();

        if (areFrequenciesRestored) {
            //Delete restored mesh
            mainWindow.deleteDrawableObject(&drawableRestoredMesh);

            drawableRestoredMesh.clear();

            //Delete cut components
            if (areComponentsCut) {
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
                if (areResultsExtracted) {
                    //Delete surfaces
                    mainWindow.deleteDrawableObject(&drawableSurfacesContainer);
                    drawableSurfacesContainer.clear();

                    drawableSurfaces.clear();

                    if (data.minSurface.numberFaces() > 0) {
                        mainWindow.deleteDrawableObject(&drawableMinSurface);
                        drawableMinSurface.clear();
                    }
                    if (data.maxSurface.numberFaces() > 0) {
                        mainWindow.deleteDrawableObject(&drawableMaxSurface);
                        drawableMaxSurface.clear();
                    }

                    //Delete stocks
                    mainWindow.deleteDrawableObject(&drawableStocksContainer);
                    drawableStocksContainer.clear();                    

                    //Delete results
                    mainWindow.deleteDrawableObject(&drawableResultsContainer);
                    drawableResultsContainer.clear();
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
    else if (ui->targetDirectionsRadio->isChecked()) {
        ui->visualizationSlider->setMinimum(0);
        ui->visualizationSlider->setMaximum(data.targetDirections.size());
        ui->visualizationSlider->setValue(0);
        ui->showNonVisibleCheck->setEnabled(true);
    }
    else if (ui->associationRadio->isChecked()) {
        ui->visualizationSlider->setMinimum(0);
        ui->visualizationSlider->setMaximum(data.targetDirections.size());
        ui->visualizationSlider->setValue(0);
        ui->showNonVisibleCheck->setEnabled(true);
    }

    updateVisualizationColors();
}

/**
 * @brief Colorize mesh depending on which radio button you have chosen
 */
void FourAxisFabricationManager::updateVisualizationColors() {
    if (ui->meshRadio->isChecked()) {
        colorizeMesh();
    }
    else if (ui->extremesRadio->isChecked()) {
        colorizeExtremes();
    }
    else if (ui->visibilityRadio->isChecked()) {
        colorizeVisibility();
    }
    else if (ui->targetDirectionsRadio->isChecked()) {
        colorizeTargetDirections();
    }
    else if (ui->associationRadio->isChecked()) {
        colorizeAssociation();
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
 * @brief Colorize face visibility from the target directions
 */
void FourAxisFabricationManager::colorizeTargetDirections() {
    unsigned int sliderValue = (unsigned int) ui->visualizationSlider->value();
    unsigned int showNonVisible = (unsigned int) ui->showNonVisibleCheck->isChecked();

    //Set default color
    drawableSmoothedMesh.setFaceColor(cg3::Color(128,128,128));

    //Information on visibility
    if (sliderValue == 0) {
        showCurrentStatusDescription();
    }
    else {
        //Subdivisions for colors
        int subd = 255 / (data.directions.size()-1);

        //Get index of the current direction
        unsigned int chosenDirectionIndex = data.targetDirections[sliderValue - 1];

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
    if (areFrequenciesRestored) {
        colorizeAssociation(drawableRestoredMesh, data.restoredMeshAssociation, data.targetDirections, data.restoredMeshNonVisibleFaces);
    }

    //Coloring cut components
    if (areComponentsCut) {
        colorizeAssociation(drawableMinComponent, data.minComponentAssociation, data.targetDirections, data.minComponentNonVisibleFaces);
        colorizeAssociation(drawableMaxComponent, data.maxComponentAssociation, data.targetDirections, data.maxComponentNonVisibleFaces);
        colorizeAssociation(drawableFourAxisComponent, data.fourAxisComponentAssociation, data.targetDirections, data.fourAxisComponentNonVisibleFaces);
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
 * @brief Show description of the current status
 */
void FourAxisFabricationManager::showCurrentStatusDescription()
{
    if (areTargetDirectionsFound) {
        std::stringstream ss;

        //Description
        ss << "Target directions: " << data.targetDirections.size() << " directions.";

        //Non-visible data
        ss << " Non-visible: " << data.nonVisibleFaces.size() << ".";
        if (isAssociationOptimized)
            ss << " Ass: " << data.associationNonVisibleFaces.size() << ".";
        if (isVisibilityRecheckedAfterRestore)
            ss << " Freq: "<<  data.restoredMeshNonVisibleFaces.size() << ".";

        //Update description label
        std::string description = ss.str();
        ui->descriptionLabel->setText(QString::fromStdString(description));
    }
}


/* ----- UI SLOTS MESH ------ */

void FourAxisFabricationManager::on_loadMeshButton_clicked()
{
    if (!isMeshLoaded) {
        std::string meshFile;
        std::string smoothedFile;

        //Get loading dialog
        meshFile = loaderSaverObj.loadDialog("Load mesh");

        if (meshFile != "") {
            isMeshLoaded = originalMesh.loadFromObj(meshFile);

            //If the mesh has been successfully loaded
            if (isMeshLoaded){
                std::string rawname, ext;
                cg3::separateExtensionFromFilename(meshFile, rawname, ext);

                //Find smoothed mesh in the path
                smoothedFile = rawname + "_smooth" + ext;
                isMeshLoaded = smoothedMesh.loadFromObj(smoothedFile);

                //If a smoothed mesh has not been found
                if (!isMeshLoaded){
                    //Get loading dialog
                    smoothedFile = loaderSaverObj.loadDialog("Load smoothed mesh");
                    if (smoothedFile != ""){
                        isMeshLoaded = smoothedMesh.loadFromObj(smoothedFile);
                    }
                }

                //If a smoothed mesh has been found
                if (isMeshLoaded) {
                    std::string meshName = meshFile.substr(meshFile.find_last_of("/") + 1);
                    addDrawableMeshes(meshName);

                    //Add meshes to the canvas, hiding the original one
                    mainWindow.setDrawableObjectVisibility(&drawableOriginalMesh, false);

                    loadedMeshFile = meshFile;
                    loadedSmoothedMeshFile = smoothedFile;

                    //Visualize mesh
                    ui->meshRadio->setChecked(true);
                    initializeVisualizationSlider();

                    //Update canvas and fit the scene
                    mainWindow.canvas.update();
                    mainWindow.canvas.fitScene();

                    std::cout << std::endl;

                    std::cout << "Mesh file: \"" << meshFile << "\"" << std::endl <<
                                 "Number of faces: " << originalMesh.numberFaces() << std::endl;
                    std::cout << "Smoothed mesh file: \"" << loadedSmoothedMeshFile << "\"" << std::endl <<
                                 "Number of faces: " << smoothedMesh.numberFaces() << std::endl;
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
    if (isMeshLoaded) {        
        //Delete objects from the canvas
        deleteDrawableObjects();

        //Clear four axis fabrication data
        clearData();

        //Clear file names
        loadedMeshFile = "";
        loadedSmoothedMeshFile = "";

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
    if (isMeshLoaded) {       
        //Delete drawable Objects
        deleteDrawableObjects();

        //Clear four axis fabrication data
        clearData();


        //Try to reload the meshes
        isMeshLoaded =
                originalMesh.loadFromObj(loadedMeshFile) &
                smoothedMesh.loadFromObj(loadedSmoothedMeshFile);

        //If the meshes have been successfully loaded
        if (isMeshLoaded){
            std::string meshName = loadedMeshFile.substr(loadedMeshFile.find_last_of("/") + 1);
            addDrawableMeshes(meshName);

            //Add meshes to the canvas, hiding the original one
            mainWindow.setDrawableObjectVisibility(&drawableOriginalMesh, false);
        }
        else {
            clearData();
        }

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
        originalMesh.setVertexColor(128,128,128);
        smoothedMesh.setVertexColor(128,128,128);

        originalMesh.saveOnObj(rawname + "_original.obj");
        smoothedMesh.saveOnObj(rawname + "_result.obj");

        if (isAssociationComputed) {
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

            if (areComponentsCut) {
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

                if (areResultsExtracted) {
                    for (size_t i = 0; i < data.surfaces.size(); i++) {
                        cg3::EigenMesh& mesh = data.surfaces[i];
                        mesh.setVertexColor(128,128,128);
                        mesh.saveOnObj(rawname + "_surface_" + std::to_string(i) + ".obj");
                    }

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


void FourAxisFabricationManager::on_targetDirectionsButton_clicked()
{
    //Get target milling directions
    getTargetDirections();

    //Update canvas and fit the scene
    mainWindow.canvas.update();
    mainWindow.canvas.fitScene();

    //Visualize target directions
    ui->targetDirectionsRadio->setChecked(true);
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
    ui->associationRadio->setChecked(true);
    initializeVisualizationSlider();

    updateUI();
}

/* ----- UI SLOTS TRANSFORMATIONS ------ */


void FourAxisFabricationManager::on_centerOnOriginButton_clicked() {
    if (isMeshLoaded){
        //Translation of the mesh
        originalMesh.translate(-smoothedMesh.boundingBox().center());
        smoothedMesh.translate(-smoothedMesh.boundingBox().center());

        //Update canvas and fit the scene
        mainWindow.canvas.update();
        mainWindow.canvas.fitScene();
    }
}

void FourAxisFabricationManager::on_plusXButton_clicked() {
    if (isMeshLoaded){
        //Translation of the mesh
        originalMesh.translate(cg3::Pointd(ui->stepSpinBox->value(), 0, 0));
        smoothedMesh.translate(cg3::Pointd(ui->stepSpinBox->value(), 0, 0));

        //Update canvas and fit the scene
        mainWindow.canvas.update();
        mainWindow.canvas.fitScene();
    }
}

void FourAxisFabricationManager::on_minusXButton_clicked() {
    if (isMeshLoaded){
        //Translation of the mesh
        originalMesh.translate(cg3::Pointd(-ui->stepSpinBox->value(), 0, 0));
        smoothedMesh.translate(cg3::Pointd(-ui->stepSpinBox->value(), 0, 0));

        //Update canvas and fit the scene
        mainWindow.canvas.update();
        mainWindow.canvas.fitScene();
    }
}

void FourAxisFabricationManager::on_plusYButton_clicked() {
    if (isMeshLoaded){
        //Translation of the mesh
        originalMesh.translate(cg3::Pointd(0, ui->stepSpinBox->value(), 0));
        smoothedMesh.translate(cg3::Pointd(0, ui->stepSpinBox->value(), 0));

        //Update canvas and fit the scene
        mainWindow.canvas.update();
        mainWindow.canvas.fitScene();
    }
}

void FourAxisFabricationManager::on_minusYButton_clicked() {
    if (isMeshLoaded){
        //Translation of the mesh
        originalMesh.translate(cg3::Pointd(0, -ui->stepSpinBox->value(), 0));
        smoothedMesh.translate(cg3::Pointd(0, -ui->stepSpinBox->value(), 0));

        //Update canvas and fit the scene
        mainWindow.canvas.update();
        mainWindow.canvas.fitScene();
    }
}

void FourAxisFabricationManager::on_plusZButton_clicked() {
    if (isMeshLoaded){
        //Translation of the mesh
        originalMesh.translate(cg3::Pointd(0, 0, ui->stepSpinBox->value()));
        smoothedMesh.translate(cg3::Pointd(0, 0, ui->stepSpinBox->value()));

        //Update canvas and fit the scene
        mainWindow.canvas.update();
        mainWindow.canvas.fitScene();
    }
}

void FourAxisFabricationManager::on_minusZButton_clicked() {
    if (isMeshLoaded){
        //Translation of the mesh
        originalMesh.translate(cg3::Pointd(0, 0, -ui->stepSpinBox->value()));
        smoothedMesh.translate(cg3::Pointd(0, 0, -ui->stepSpinBox->value()));

        //Update canvas and fit the scene
        mainWindow.canvas.update();
        mainWindow.canvas.fitScene();
    }
}



void FourAxisFabricationManager::on_rotateButton_clicked() {
    if (isMeshLoaded){
        //Rotation of the mesh
        cg3::Vec3 axis(ui->axisXSpinBox->value(), ui->axisYSpinBox->value(), ui->axisZSpinBox->value());
        double angle = ui->angleSpinBox->value() * M_PI/180;

        Eigen::Matrix3d m;
        cg3::rotationMatrix(axis, angle, m);

        originalMesh.rotate(m);
        smoothedMesh.rotate(m);

        //Update canvas and fit the scene
        mainWindow.canvas.update();
        mainWindow.canvas.fitScene();
    }
}




void FourAxisFabricationManager::on_scaleButton_clicked() {
    if (isMeshLoaded){
        //Scale the mesh
        cg3::Vec3 scaleFactor(ui->scaleXSpinBox->value(), ui->scaleYSpinBox->value(), ui->scaleZSpinBox->value());

        originalMesh.scale(scaleFactor);
        smoothedMesh.scale(scaleFactor);

        //Update canvas and fit the scene
        mainWindow.canvas.update();
        mainWindow.canvas.fitScene();
    }
}

void FourAxisFabricationManager::on_inverseScaleButton_clicked() {
    if (isMeshLoaded){
        //Scale the mesh
        cg3::Vec3 scaleFactor(1.0/ui->scaleXSpinBox->value(), 1.0/ui->scaleYSpinBox->value(), 1.0/ui->scaleZSpinBox->value());

        originalMesh.scale(scaleFactor);
        smoothedMesh.scale(scaleFactor);

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

void FourAxisFabricationManager::on_targetDirectionsRadio_clicked() {
    initializeVisualizationSlider();
}

void FourAxisFabricationManager::on_associationRadio_clicked() {
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
    updateVisualizationColors();

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
        else if (ui->targetDirectionsRadio->isChecked() || ui->associationRadio->isChecked()) {
            if (sliderValue > 0) {
                unsigned int currentIndex = data.targetDirections[sliderValue - 1];
                cg3::Vec3 currentDirection = data.directions[currentIndex];
                setCameraDirection(-currentDirection);
            }
            else {
                resetCameraDirection();
            }
        }
    }
}
