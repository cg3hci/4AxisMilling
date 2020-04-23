/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#include "fafmanager.h"
#include "ui_fafmanager.h"

#include <sstream>
#include <string>
#include <fstream>

#include <cg3/geometry/transformations3.h>

#include <cg3/utilities/string.h>
#include <cg3/utilities/timer.h>

#include <cg3/libigl/mesh_distance.h>

#include <cg3/algorithms/normalization.h>
#include <cg3/algorithms/saliency.h>


/* ----- CONSTRUCTORS/DESTRUCTOR ------ */

/**
 * @brief Default constructor
 * @param parent Parent widget
 */
FAFManager::FAFManager(QWidget *parent) :
    QFrame(parent),
    ui(new Ui::FAFManager),
    mainWindow((cg3::viewer::MainWindow&)*parent)
{
    ui->setupUi(this);

    initialize();
}


/**
 * @brief Destructor
 */
FAFManager::~FAFManager(){
    delete ui;
}




/* ----- UI METHODS ------ */

/**
 * @brief Initialization of the manager
 */
void FAFManager::initialize() {
    loaderSaverObj.addSupportedExtension("obj");
    loaderSaverData.addSupportedExtension("faf");

    mainWindow.canvas.setOrthographicCamera();

    QObject::connect(
            &mainWindow.canvas, SIGNAL(objectPicked(const cg3::PickableObject*, unsigned int)),
            this, SLOT(facePicked(const cg3::PickableObject*, unsigned int)));
    mainWindow.canvas.setMouseBinding(Qt::ControlModifier, Qt::LeftButton, cg3::viewer::GLCanvas::SELECT);

    clearData();

    updateUI();
}

/**
 * @brief Update UI depending on the current state
 */
void FAFManager::updateUI() {
    // ----- Mesh loading -----
    ui->loadMeshButton->setEnabled(!data.isMeshLoaded);
    ui->clearMeshButton->setEnabled(data.isMeshLoaded);
    ui->reloadMeshButton->setEnabled(data.isMeshLoaded);
    ui->saveResultsButton->setEnabled(data.isMeshLoaded);
    ui->loadDataButton->setEnabled(!data.isMeshLoaded);
    ui->saveDataButton->setEnabled(data.isMeshLoaded);


    // ----- Four axis fabrication -----
    ui->fourAxisFabricationGroup->setEnabled(data.isMeshLoaded);

    //Painting

    //Scale and stock generation
    ui->scaleStockButton->setEnabled(!data.isMeshScaledAndStockGenerated);
    ui->stockLengthLabel->setEnabled(!data.isMeshScaledAndStockGenerated);
    ui->stockLengthSpinBox->setEnabled(!data.isMeshScaledAndStockGenerated);
    ui->stockDiameterLabel->setEnabled(!data.isMeshScaledAndStockGenerated);
    ui->stockDiameterSpinBox->setEnabled(!data.isMeshScaledAndStockGenerated);
    ui->modelScaleCheckBox->setEnabled(!data.isMeshScaledAndStockGenerated);
    ui->modelLengthSpinBox->setEnabled(!data.isMeshScaledAndStockGenerated);

    //Saliency
    ui->saliencyFindDetailsButton->setEnabled(!data.isSaliencyComputed);
    ui->saliencyCheckBox->setEnabled(!data.isSaliencyComputed);
    ui->saliencyRingLabel->setEnabled(!data.isSaliencyComputed);
    ui->saliencyRingSpinBox->setEnabled(!data.isSaliencyComputed);
    ui->saliencyUnitScaleCheckBox->setEnabled(!data.isSaliencyComputed);
    ui->saliencyScalesLabel->setEnabled(!data.isSaliencyComputed);
    ui->saliencyScalesSpinBox->setEnabled(!data.isSaliencyComputed);
    ui->saliencyEpsLabel->setEnabled(!data.isSaliencyComputed);
    ui->saliencyEpsSpinBox->setEnabled(!data.isSaliencyComputed);
    ui->saliencyLaplacianSmoothingLabel->setEnabled(!data.isSaliencyComputed);
    ui->saliencyLaplacianSmoothingSpinBox->setEnabled(!data.isSaliencyComputed);
    ui->saliencyMaxSmoothingLabel->setEnabled(!data.isSaliencyComputed);
    ui->saliencyMaxSmoothingSpinBox->setEnabled(!data.isSaliencyComputed);

    //Detail painting
    ui->paintingSizeLabel->setEnabled(data.isSaliencyComputed && !data.isMeshSmoothed);
    ui->paintingSizeSlider->setEnabled(data.isSaliencyComputed && !data.isMeshSmoothed);
    ui->paintingFillRadioBox->setEnabled(data.isSaliencyComputed && !data.isMeshSmoothed);
    ui->paintingFreeRadioBox->setEnabled(data.isSaliencyComputed && !data.isMeshSmoothed);
    ui->paintingResetRadioBox->setEnabled(data.isSaliencyComputed && !data.isMeshSmoothed);
    ui->paintingFillValueLabel->setEnabled(data.isSaliencyComputed && !data.isMeshSmoothed);
    ui->paintingFillValueSpinBox->setEnabled(data.isSaliencyComputed && !data.isMeshSmoothed);

    //Smoothing
    ui->smoothingButton->setEnabled(!data.isMeshSmoothed);
    ui->smoothingIterationsLabel->setEnabled(!data.isMeshSmoothed);
    ui->smoothingIterationsSpinBox->setEnabled(!data.isMeshSmoothed);
    ui->smoothingLambdaLabel->setEnabled(!data.isMeshSmoothed);
    ui->smoothingLambdaSpinBox->setEnabled(!data.isMeshSmoothed);
    ui->smoothingMuLabel->setEnabled(!data.isMeshSmoothed);
    ui->smoothingMuSpinBox->setEnabled(!data.isMeshSmoothed);

    //Optimal orientation
    ui->optimalOrientationButton->setEnabled(!data.isMeshOriented);
    ui->optimalOrientationOrientationsLabel->setEnabled(!data.isMeshOriented);
    ui->optimalOrientationOrientationsSpinBox->setEnabled(!data.isMeshOriented);
    ui->optimalOrientationDeterministicCheckBox->setEnabled(!data.isMeshOriented);
    ui->optimalOrientationExtremeWeightLabel->setEnabled(!data.isMeshOriented);
    ui->optimalOrientationExtremeWeightSpinBox->setEnabled(!data.isMeshOriented);
    ui->optimalOrientationBBWeightLabel->setEnabled(!data.isMeshOriented);
    ui->optimalOrientationBBWeightSpinBox->setEnabled(!data.isMeshOriented);

    //Select extremes
    ui->heightfieldAngleLabel->setEnabled(!data.areExtremesSelected);
    ui->heightfieldAngleSpinBox->setEnabled(!data.areExtremesSelected);
    ui->selectExtremesButton->setEnabled(!data.areExtremesSelected);

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
    ui->getAssociationDetailMultiplierSpinBox->setEnabled(!data.isAssociationComputed);
    ui->getAssociationDetailMultiplierLabel->setEnabled(!data.isAssociationComputed);
    ui->getAssociationCompactnessLabel->setEnabled(!data.isAssociationComputed);
    ui->getAssociationCompactnessSpinBox->setEnabled(!data.isAssociationComputed);
    ui->getAssociationFixExtremesCheckBox->setEnabled(!data.isAssociationComputed);

    //Optimization
    ui->optimizationButton->setEnabled(!data.isAssociationOptimized);
    ui->optimizationRelaxHolesCheckBox->setEnabled(!data.isAssociationOptimized);
    ui->optimizationLoseHolesCheckBox->setEnabled(!data.isAssociationOptimized);
    ui->optimizationMinChartAreaLabel->setEnabled(!data.isAssociationOptimized);
    ui->optimizationMinChartAreaSpinBox->setEnabled(!data.isAssociationOptimized);

    //Smooth lines
    ui->smoothLinesButton->setEnabled(!data.isLineSmoothed);
    ui->smoothLinesCheckBox->setEnabled(!data.isLineSmoothed);

    //Restore frequencies
    ui->restoreFrequenciesButton->setEnabled(!data.areFrequenciesRestored);
    ui->restoreFrequenciesIterationsLabel->setEnabled(!data.areFrequenciesRestored);
    ui->restoreFrequenciesIterationsSpinBox->setEnabled(!data.areFrequenciesRestored);
    ui->restoreFrequenciesRecheckCheckBox->setEnabled(!data.areFrequenciesRestored);
    ui->restoreFrequenciesReassignNonVisibleCheckBox->setEnabled(!data.areFrequenciesRestored);

    //Cut components
    ui->cutComponentsButton->setEnabled(!data.areComponentsCut);
    ui->cutComponentsCheckBox->setEnabled(!data.areComponentsCut);

    //Extract results
    ui->extractResultsButton->setEnabled(!data.areResultsExtracted);
    ui->extractResultsFirstLayerAngleLabel->setEnabled(!data.areResultsExtracted);
    ui->extractResultsFirstLayerAngleSpinBox->setEnabled(!data.areResultsExtracted);
    ui->extractResultsFirstLayerOffsetLabel->setEnabled(!data.areResultsExtracted);
    ui->extractResultsFirstLayerOffsetSpinBox->setEnabled(!data.areResultsExtracted);
    ui->extractResultsFirstLayerSmoothingIterationsLabel->setEnabled(!data.areResultsExtracted);
    ui->extractResultsFirstLayerSmoothingIterationsSpinBox->setEnabled(!data.areResultsExtracted);
    ui->extractResultsFirstLayerSmoothingWeightLabel->setEnabled(!data.areResultsExtracted);
    ui->extractResultsFirstLayerSmoothingWeightSpinBox->setEnabled(!data.areResultsExtracted);
    ui->extractResultsSecondLayerStepWidthLabel->setEnabled(!data.areResultsExtracted);
    ui->extractResultsSecondLayerStepWidthSpinBox->setEnabled(!data.areResultsExtracted);
    ui->extractResultsSecondLayerStepHeightLabel->setEnabled(!data.areResultsExtracted);
    ui->extractResultsSecondLayerStepHeightSpinBox->setEnabled(!data.areResultsExtracted);
    ui->extractResultsSecondLayerSideSubdivisionCheckBox->setEnabled(!data.areResultsExtracted);
    ui->extractResultsRotateCheckBox->setEnabled(!data.areResultsExtracted);
    ui->extractResultsXDirectionsOrderLabel->setEnabled(!data.areResultsExtracted);
    ui->extractResultsXDirectionsOrderFrame->setEnabled(!data.areResultsExtracted);
    ui->extractResultsMinFirstCheckBox->setEnabled(!data.areResultsExtracted);



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

    ui->modelScaleCheckBox->setFocus();
}

/**
 * @brief Clear data of four axis fabrication
 */
void FAFManager::clearData() {
    data.clear();
}




/* ----- COMPUTING METHODS ------ */

/**
 * @brief Scale mesh and generate stock
 */
void FAFManager::scaleAndStock() {
    if (!data.isMeshScaledAndStockGenerated) {
        std::cout << std::endl << "#######################################################################" << std::endl << std::endl;

        //Get UI data
        bool scaleModel = ui->modelScaleCheckBox->isChecked();
        double modelLength = ui->modelLengthSpinBox->value();
        double stockLength = ui->stockLengthSpinBox->value();
        double stockDiameter = ui->stockDiameterSpinBox->value();

        cg3::Timer t(std::string("Scale and stock generation"));

        //Scale mesh and get stock
        FourAxisFabrication::centerAndScale(
                    data,
                    scaleModel,
                    modelLength);


        //Scale mesh and get stock
        FourAxisFabrication::generateStock(
                    data,
                    stockLength,
                    stockDiameter);

        t.stopAndPrint();

        data.isMeshScaledAndStockGenerated = true;

        addDrawableStock();
        updateDrawableMesh();
    }
}

/**
 * @brief Find details
 */
void FAFManager::findDetails() {
    if (!data.isSaliencyComputed) {
        scaleAndStock();

        //Get UI data
        bool computeBySaliency = ui->saliencyCheckBox->isChecked();
        bool unitScale = ui->saliencyUnitScaleCheckBox->isChecked();
        unsigned int nRing = ui->saliencyRingSpinBox->value();
        unsigned int nScales = ui->saliencyScalesSpinBox->value();
        double eps = ui->saliencyEpsSpinBox->value();
        unsigned int maxIterations = ui->saliencyMaxSmoothingSpinBox->value();
        unsigned int laplacianIterations = ui->saliencyLaplacianSmoothingSpinBox->value();

        cg3::Timer t(std::string("Saliency details"));

        //Find details
        FourAxisFabrication::findDetails(data, unitScale, nRing, nScales, eps, computeBySaliency, maxIterations, laplacianIterations);

        t.stopAndPrint();

        data.isSaliencyComputed = true;

        addDrawableDetailMesh();
        colorizeDetailMesh();
    }
}

/**
 * @brief Compute smoothing
 */
void FAFManager::smoothing() {
    if (!data.isMeshSmoothed) {
        findDetails();

        //Get UI data
        int iterations = ui->smoothingIterationsSpinBox->value();
        float lambda = ui->smoothingLambdaSpinBox->value();
        float mu = ui->smoothingMuSpinBox->value();

        cg3::Timer t(std::string("Smoothing"));

        //Get smoothed mesh
        FourAxisFabrication::smoothing(
                    data,
                    iterations,
                    lambda,
                    mu);

        t.stopAndPrint();

        data.isMeshSmoothed = true;

        addDrawableSmoothedMesh();
    }
}


/**
 * @brief Compute optimal orientation
 */
void FAFManager::optimalOrientation() {
    if (!data.isMeshOriented) {
        smoothing();

        //Get UI data
        double stockLength = ui->stockLengthSpinBox->value();
        double stockDiameter = ui->stockDiameterSpinBox->value();
        unsigned int nOrientations = (unsigned int) ui->optimalOrientationOrientationsSpinBox->value();
        double extremeWeight = (double) ui->optimalOrientationExtremeWeightSpinBox->value();
        double BBWeight = (double) ui->optimalOrientationBBWeightSpinBox->value();
        bool deterministic = ui->optimalOrientationDeterministicCheckBox->isChecked();

        cg3::Timer t(std::string("Optimal orientation"));

        //Get optimal mesh orientation
        bool res = FourAxisFabrication::rotateToOptimalOrientation(
                    data.mesh,
                    data.smoothedMesh,
                    stockLength,
                    stockDiameter,
                    nOrientations,
                    extremeWeight,
                    BBWeight,
                    deterministic);


        t.stopAndPrint();

        if (res) {
            data.isMeshOriented = true;

            updateDrawableMesh();
            updateDrawableSmoothedMesh();
        }
        else {
            QMessageBox::warning(this, tr("Warning"), tr("Model cannot fit!"));
            on_reloadMeshButton_clicked();
        }
    }
}

/**
 * @brief Select extremes
 */
void FAFManager::selectExtremes() {
    if (!data.areExtremesSelected) {
        optimalOrientation();

        //Get UI data
        double heightfieldAngle = ui->heightfieldAngleSpinBox->value() / 180.0 * M_PI;

        cg3::Timer t(std::string("Select extremes"));

        //Get extremes on x-axis to be selected
        FourAxisFabrication::selectExtremesOnXAxis(data.smoothedMesh, heightfieldAngle, data);

        t.stopAndPrint();

        data.areExtremesSelected = true;
    }
}

/**
 * @brief Check visibility from various directions
 */
void FAFManager::checkVisibility() {
    if (!data.isVisibilityChecked) {
        selectExtremes();

        //Get UI data
        double heightfieldAngle = ui->heightfieldAngleSpinBox->value() / 180.0 * M_PI;
        unsigned int nDirections = (unsigned int) ui->checkVisibilityDirectionsSpinBox->value();
        unsigned int resolution = (unsigned int) ui->checkVisibilityResolutionSpinBox->value();
        bool includeXDirections = ui->checkVisibilityXDirectionsCheckBox->isChecked();
        FourAxisFabrication::CheckMode checkMode =
                (ui->checkVisibilityGLRadio->isChecked() ?
                     FourAxisFabrication::OPENGL :
                     ui->checkVisibilityRayRadio->isChecked() ?
                         FourAxisFabrication::RAYSHOOTING :
                         FourAxisFabrication::PROJECTION);

        cg3::Timer t(std::string("Visibility check"));

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
void FAFManager::getAssociation() {
    if (!data.isAssociationComputed) {
        checkVisibility();

        //Get UI data
        double dataSigma = ui->getAssociationDataSigmaSpinBox->value();
        double detailMultiplier = ui->getAssociationDetailMultiplierSpinBox->value();
        double compactness = ui->getAssociationCompactnessSpinBox->value();
        bool fixExtremes = ui->getAssociationFixExtremesCheckBox->isChecked();

        cg3::Timer t(std::string("Get association"));

        //Get association
        FourAxisFabrication::getAssociation(
                    data.smoothedMesh,
                    dataSigma,
                    detailMultiplier,
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
void FAFManager::optimizeAssociation() {
    if (!data.isAssociationOptimized) {
        getAssociation();

        //Get UI data
        bool relaxHoles = ui->optimizationRelaxHolesCheckBox->isChecked();
        bool loseHoles = ui->optimizationLoseHolesCheckBox->isChecked();
        double minChartArea = ui->optimizationMinChartAreaSpinBox->value();

        cg3::Timer t(std::string("Optimize association"));

        //Execute optimization
        FourAxisFabrication::optimization(
                    data.smoothedMesh,
                    relaxHoles,
                    loseHoles,
                    minChartArea,
                    data);

        t.stopAndPrint();

        data.isAssociationOptimized = true;

        updateDrawableSmoothedMesh();

        std::cout << "Non-visible triangles after optimization: " << data.associationNonVisibleFaces.size() << std::endl;
    }
}

/**
 * @brief Get optimized association
 */
void FAFManager::smoothLines() {
    if (!data.isLineSmoothed) {
        optimizeAssociation();

        //Get UI data;
        bool smoothEdgeLines = ui->smoothLinesCheckBox->isChecked();

        cg3::Timer t(std::string("Smooth lines"));

        //Execute optimization
        FourAxisFabrication::smoothLines(
                    data.smoothedMesh,
                    smoothEdgeLines,
                    data);

        t.stopAndPrint();

        data.isLineSmoothed = true;

        updateDrawableSmoothedMesh();

        std::cout << "Non-visible triangles after line smoothings: " << data.associationNonVisibleFaces.size() << std::endl;
    }
}


/**
 * @brief Restore frequencies
 */
void FAFManager::restoreFrequencies() {
    if (!data.areFrequenciesRestored) {
        smoothLines();

        //Get UI data
        double heightfieldAngle = ui->heightfieldAngleSpinBox->value() / 180.0 * M_PI;
        unsigned int nIterations = (unsigned int) ui->restoreFrequenciesIterationsSpinBox->value();

        //Get UI data
        unsigned int resolution = (unsigned int) ui->checkVisibilityResolutionSpinBox->value();
        bool includeXDirections = ui->checkVisibilityXDirectionsCheckBox->isChecked();
        bool recheck = ui->restoreFrequenciesRecheckCheckBox->isChecked();
        bool reassign = ui->restoreFrequenciesReassignNonVisibleCheckBox->isChecked();
        FourAxisFabrication::CheckMode checkMode =
                (ui->checkVisibilityGLRadio->isChecked() ?
                     FourAxisFabrication::OPENGL :
                     ui->checkVisibilityRayRadio->isChecked() ?
                         FourAxisFabrication::RAYSHOOTING :
                         FourAxisFabrication::PROJECTION);


        double haussDistance = cg3::libigl::hausdorffDistance(data.mesh, data.smoothedMesh);
        cg3::BoundingBox3 originalMeshBB = data.mesh.boundingBox();
        double haussDistanceBB = haussDistance/originalMeshBB.diag();

        std::cout << "Smoothed -> Haussdorff distance: " << haussDistance << " (w.r.t. bounding box: " << haussDistanceBB << ")" << std::endl;

        cg3::Timer t(std::string("Restore frequencies"));

        //Restore frequencies
        FourAxisFabrication::restoreFrequencies(nIterations, heightfieldAngle, data.mesh, data.smoothedMesh, data);

        t.stopAndPrint();



        haussDistance = cg3::libigl::hausdorffDistance(data.mesh, data.restoredMesh);
        originalMeshBB = data.mesh.boundingBox();
        haussDistanceBB = haussDistance/originalMeshBB.diag();

        std::cout << "Restored -> Haussdorff distance: " << haussDistance << " (w.r.t. bounding box: " << haussDistanceBB << ")" << std::endl;



        cg3::Timer tCheck("Recheck visibility after frequencies have been restored");

        //Check if it is a valid association
        FourAxisFabrication::recheckVisibilityAfterRestore(recheck, resolution, heightfieldAngle, includeXDirections, reassign, data, checkMode);

        tCheck.stopAndPrint();

        std::cout << "Non-visible triangles after recheck: " << data.restoredMeshNonVisibleFaces.size() << std::endl;



        data.areFrequenciesRestored = true;

        addDrawableRestoredMesh();
    }
}

/**
 * @brief Cut components
 */
void FAFManager::cutComponents() {
    if (!data.areComponentsCut) {
        restoreFrequencies();

        //Get UI data
        bool cutComponents = ui->cutComponentsCheckBox->isChecked();



        cg3::Timer t(std::string("Cut components"));

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
void FAFManager::extractResults() {
    if (!data.areResultsExtracted) {
        cutComponents();

        //Get UI data
        double heightfieldAngle = ui->heightfieldAngleSpinBox->value() / 180.0 * M_PI;
        double stockLength = ui->stockLengthSpinBox->value();
        double stockDiameter = ui->stockDiameterSpinBox->value();
        double firstLayerAngle = ui->extractResultsFirstLayerAngleSpinBox->value() / 180.0 * M_PI;
        double firstLayerOffset = ui->extractResultsFirstLayerOffsetSpinBox->value();
        unsigned int smoothingIterations = static_cast<unsigned int>(ui->extractResultsFirstLayerSmoothingIterationsSpinBox->value());
        double smoothingWeight = ui->extractResultsFirstLayerSmoothingWeightSpinBox->value();
        double secondLayerStepWidth = ui->extractResultsSecondLayerStepWidthSpinBox->value();
        double secondLayerStepHeight = ui->extractResultsSecondLayerStepHeightSpinBox->value();
        bool secondLayerSideSubdivision = ui->extractResultsSecondLayerSideSubdivisionCheckBox->isChecked();
        bool rotateResults = ui->extractResultsRotateCheckBox->isChecked();
        bool xDirectionsAfter = ui->extractResultsXDirectionsAfterRadio->isChecked();
        bool minFirst = ui->extractResultsMinFirstCheckBox->isChecked();

        cg3::Timer t(std::string("Extract results"));


        //Extract results
        FourAxisFabrication::extractResults(data, stockLength, stockDiameter, firstLayerAngle, firstLayerOffset, smoothingIterations, smoothingWeight, secondLayerStepWidth, secondLayerStepHeight, secondLayerSideSubdivision, heightfieldAngle, xDirectionsAfter, minFirst, rotateResults);

        t.stopAndPrint();

        data.areResultsExtracted = true;

        addDrawableResults();
    }
}




/* ----- VISUALIZATION METHODS ------ */

/**
 * @brief Add drawable meshes
 */
void FAFManager::addDrawableMesh() {
    //Add drawable meshes to the canvas
    drawableOriginalMesh = cg3::DrawableEigenMesh(data.mesh);
    drawableOriginalMesh.setFlatShading();

    mainWindow.pushDrawableObject(&drawableOriginalMesh, "Mesh", true);
}

/**
 * @brief Add drawable stock
 */
void FAFManager::addDrawableStock() {
    drawableStock = cg3::DrawableEigenMesh(data.stock);
    drawableStock.setFlatShading();

    mainWindow.pushDrawableObject(&drawableStock, "Stock");

    mainWindow.setDrawableObjectVisibility(&drawableStock, false);
}

/**
 * @brief Add detail mesh
 */
void FAFManager::addDrawableDetailMesh() {
    drawableDetailMesh = cg3::DrawableEigenMesh(data.mesh);
    drawableDetailMesh.setFlatShading();

    cg3::libigl::heatGeodesicsPrecomputeData(drawableDetailMesh, detailMeshGeodesicsData);

    mainWindow.pushDrawableObject(&drawableDetailMesh, "Detail mesh");

    mainWindow.setDrawableObjectVisibility(&drawableOriginalMesh, false);
}

/**
 * @brief Add drawable smoothed mesh
 */
void FAFManager::addDrawableSmoothedMesh() {
    drawableSmoothedMesh = cg3::DrawableEigenMesh(data.smoothedMesh);
    drawableSmoothedMesh.setFlatShading();

    mainWindow.pushDrawableObject(&drawableSmoothedMesh, "Smoothed mesh");

    mainWindow.setDrawableObjectVisibility(&drawableOriginalMesh, false);
    mainWindow.setDrawableObjectVisibility(&drawableDetailMesh, false);
}

/**
 * @brief Add drawable restored mesh to the canvas
 */
void FAFManager::addDrawableRestoredMesh() {
    //Hide smoothed mesh
    mainWindow.setDrawableObjectVisibility(&drawableOriginalMesh, false);
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
void FAFManager::addDrawableCutComponents() {
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
void FAFManager::addDrawableResults() {
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
void FAFManager::updateDrawableMesh() {
    //Update drawable meshes (already in the canvas)
    bool meshOriginalVisibility = drawableOriginalMesh.isVisible();

    drawableOriginalMesh = cg3::DrawableEigenMesh(data.mesh);
    drawableOriginalMesh.setFlatShading();

    mainWindow.setDrawableObjectVisibility(&drawableOriginalMesh, meshOriginalVisibility);
}

/**
 * @brief Update smoothed meshes
 */
void FAFManager::updateDrawableSmoothedMesh() {
    //Update drawable meshes (already in the canvas)
    bool smoothVisibility = drawableSmoothedMesh.isVisible();

    drawableSmoothedMesh = cg3::DrawableEigenMesh(data.smoothedMesh);
    drawableSmoothedMesh.setFlatShading();

    mainWindow.setDrawableObjectVisibility(&drawableSmoothedMesh, smoothVisibility);
}

/**
 * @brief Update drawable meshes
 */
void FAFManager::updateDrawableRestoredMesh() {
    //Update drawable meshes (already in the canvas)
    bool restoredVisibility = drawableRestoredMesh.isVisible();

    drawableRestoredMesh = cg3::DrawableEigenMesh(data.restoredMesh);
    drawableRestoredMesh.setFlatShading();

    mainWindow.setDrawableObjectVisibility(&drawableRestoredMesh, restoredVisibility);
}

/**
 * @brief Delte all drawable objects from the canvas
 */
void FAFManager::deleteDrawableObjects() {
    if (data.isMeshLoaded) {
        //Delete meshes
        mainWindow.deleteDrawableObject(&drawableOriginalMesh);
        drawableOriginalMesh.clear();
    }

    if (data.isSaliencyComputed) {
        mainWindow.deleteDrawableObject(&drawableDetailMesh);
        drawableDetailMesh.clear();
    }

    if (data.isMeshScaledAndStockGenerated) {
        mainWindow.deleteDrawableObject(&drawableStock);
        drawableStock.clear();
    }

    if (data.isMeshSmoothed) {
        mainWindow.deleteDrawableObject(&drawableSmoothedMesh);
        drawableSmoothedMesh.clear();
    }

    if (data.areFrequenciesRestored) {
        mainWindow.deleteDrawableObject(&drawableRestoredMesh);
        drawableRestoredMesh.clear();
    }

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
    }

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


/**
 * @brief Reset camera direction
 */
void FAFManager::resetCameraDirection() {
    mainWindow.canvas.setCameraDirection(cg3::Vec3d(1,0,0));
    mainWindow.canvas.setCameraDirection(cg3::Vec3d(0,0,-1));

    mainWindow.canvas.update();
    mainWindow.canvas.fitScene();
}

/**
 * @brief Set camera direction
 */
void FAFManager::setCameraDirection(const cg3::Vec3d& dir) {
    mainWindow.canvas.setCameraDirection(dir);

    mainWindow.canvas.update();
    mainWindow.canvas.fitScene();
}


/**
 * @brief Initialize the slider depending on which radio button you have chosen
 */
void FAFManager::initializeVisualizationSlider() {
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
void FAFManager::updateVisualization() {
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
void FAFManager::colorizeMesh() {
    //Set default color
    drawableSmoothedMesh.setFaceColor(cg3::Color(128,128,128));
    drawableRestoredMesh.setFaceColor(cg3::Color(128,128,128));
    drawableMinComponent.setFaceColor(cg3::Color(128,128,128));
    drawableMaxComponent.setFaceColor(cg3::Color(128,128,128));
    drawableFourAxisComponent.setFaceColor(cg3::Color(128,128,128));

    ui->descriptionLabel->setText(""); //Empty description text
}

/**
 * @brief Colorize detail mesh
 */
void FAFManager::colorizeDetailMesh()
{
    drawableDetailMesh.setFaceColor(cg3::Color(128,128,128));
    drawableDetailMesh.setVertexColor(cg3::Color(128,128,128));

    if (data.isSaliencyComputed) {
        double minSaliency = data.saliency[0];
        double maxSaliency = data.saliency[0];
        for (const double& value : data.saliency) {
            if (value >= 0 && value <= 1) {
                minSaliency = std::min(minSaliency, value);
                maxSaliency = std::max(maxSaliency, value);
            }
        }

        std::cout << "Saliency min: " << minSaliency << std::endl;
        std::cout << "Saliency max: " << maxSaliency << std::endl;

        for(size_t fId = 0; fId < data.faceSaliency.size(); fId++) {
            double normalizedValue;

            if (data.faceSaliency[fId] > maxSaliency)
                normalizedValue = 1.0;
            else if (maxSaliency - minSaliency == 0 || data.faceSaliency[fId] < 0)
                normalizedValue = 0.0;
            else
                normalizedValue = (data.faceSaliency[fId] - minSaliency) / (maxSaliency - minSaliency);

            cg3::Color color = computeColorByNormalizedValue(normalizedValue);
            drawableDetailMesh.setFaceColor(color, fId);
        }

        for(size_t vId = 0; vId < drawableDetailMesh.numberVertices(); vId++) {
            double normalizedValue;

            if (data.saliency[vId] > maxSaliency)
                normalizedValue = 1.0;
            else if (maxSaliency - minSaliency == 0 || data.saliency[vId] < 0)
                normalizedValue = 0.0;
            else
                normalizedValue = (data.saliency[vId] - minSaliency) / (maxSaliency - minSaliency);

            cg3::Color color = computeColorByNormalizedValue(normalizedValue);
            drawableDetailMesh.setVertexColor(color, vId);
        }
    }

    mainWindow.canvas.update();
}

/**
 * @brief Colorize the min and max extremes
 */
void FAFManager::colorizeExtremes() {
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
    cg3::Vec3d xAxis(1,0,0);
    cg3::Vec3d xAxisOpposite(-1,0,0);

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
void FAFManager::colorizeVisibility() {
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
void FAFManager::colorizeAssociation() {
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
void FAFManager::colorizeAssociation(
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

    //For each face of the drawable mesh
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
void FAFManager::showResults() {
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
void FAFManager::showCurrentStatusDescription()
{
    if (data.isAssociationComputed) {
        std::stringstream ss;

        //Description
        ss << "Target directions: " << data.targetDirections.size() << " directions.";

        //Non-visible data
        ss << " Non-visible: " << data.nonVisibleFaces.size() << ".";
        if (data.isAssociationOptimized)
            ss << " Ass: " << data.associationNonVisibleFaces.size() << ".";
        if (data.areFrequenciesRestored)
            ss << " Freq: "<<  data.restoredMeshNonVisibleFaces.size() << ".";

        //Update description label
        std::string description = ss.str();
        ui->descriptionLabel->setText(QString::fromStdString(description));
    }
}

cg3::Color FAFManager::computeColorByNormalizedValue(const double value)
{
    cg3::Color color(0.0, 0.0, 0.0);

    if (value <= 0) {
        color.setRedF(1.0);
    }
    else if (value >= 1) {
        color.setBlueF(1.0);
    }
    else if (value <= 0.5f) {
        double normalizedValue = value * 2.0;
        color.setRedF(1.0 - normalizedValue);
        color.setGreenF(normalizedValue);
    }
    else {
        double normalizedValue = (value - 0.5) * 2.0;
        color.setGreenF(1.0 - normalizedValue);
        color.setBlueF(normalizedValue);
    }

    return color;
}


/* ----- UI SLOTS MESH ------ */

void FAFManager::on_loadMeshButton_clicked()
{
    if (!data.isMeshLoaded) {
        std::string meshFile;

        //Get loading dialog
        meshFile = loaderSaverObj.loadDialog("Load mesh");

        if (meshFile != "") {
            data.isMeshLoaded = data.originalMesh.loadFromObj(meshFile);

            //If the mesh has been successfully loaded
            if (data.isMeshLoaded){
                std::string rawname, ext;
                cg3::separateExtensionFromFilename(meshFile, rawname, ext);

                std::string meshName = meshFile.substr(meshFile.find_last_of("/") + 1);

                data.mesh = data.originalMesh;

                addDrawableMesh();

                //Visualize mesh
                ui->meshRadio->setChecked(true);
                initializeVisualizationSlider();

                //Update canvas and fit the scene
                mainWindow.canvas.update();
                mainWindow.canvas.fitScene();

                std::cout << std::endl;

                std::cout << "Original: \"" << meshName << "\" " << "(" << data.mesh.numberFaces() << " F / " << data.mesh.numberVertices() << " V)" << std::endl;
            }
            else {
                clearData();
            }

        }

        updateUI();

        resetCameraDirection();
    }
}

void FAFManager::on_clearMeshButton_clicked()
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

void FAFManager::on_reloadMeshButton_clicked()
{
    if (data.isMeshLoaded) {
        //Delete drawable Objects
        deleteDrawableObjects();

        //Saving for reloading
        cg3::EigenMesh originalMeshCopy = data.originalMesh;

        //Clear four axis fabrication data
        clearData();

        //Reload
        data.originalMesh = originalMeshCopy;

        data.mesh = originalMeshCopy;

        data.isMeshLoaded = true;

        //Add drawable meshes
        addDrawableMesh();

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

void FAFManager::on_saveResultsButton_clicked() {
    //Get saving dialog
    std::string selectedExtension;
    std::string saveFileName = loaderSaverObj.saveDialog("Save mesh", selectedExtension);
    saveFileName += "." + selectedExtension;

    if (saveFileName != "") {
        std::string rawname, ext;
        cg3::separateExtensionFromFilename(saveFileName, rawname, ext);

        //Save on obj files
        data.mesh.saveOnObj(rawname + "_original.obj");

        if (data.isMeshSmoothed) {
            data.smoothedMesh.saveOnObj(rawname + "_smoothed.obj");

            if (data.isAssociationComputed) {
                std::ofstream resultFile;
                resultFile.open (rawname + "_directions.txt");
                for (size_t i = 0; i < data.resultsAssociation.size(); i++) {
                    size_t label = data.resultsAssociation[i];
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
}


/* ----- LOAD/SAVE DATA SLOTS ------ */

void FAFManager::on_loadDataButton_clicked()
{
    //Get loading dialog
    std::string dataFile = loaderSaverData.loadDialog("Load data");

    if (dataFile != "") {
        std::ifstream inputStream(dataFile);
        data.deserialize(inputStream);
    }

    if (data.isMeshLoaded) {
        addDrawableMesh();

        //Visualize mesh
        ui->meshRadio->setChecked(true);
    }

    if (data.isMeshScaledAndStockGenerated) {
        addDrawableStock();
    }

    if (data.isSaliencyComputed) {
        addDrawableDetailMesh();
    }

    if (data.isMeshSmoothed) {
        addDrawableSmoothedMesh();

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

void FAFManager::on_saveDataButton_clicked()
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


void FAFManager::on_scaleStockButton_clicked()
{
    //Scale and stock generation
    scaleAndStock();

    //Update canvas and fit the scene
    mainWindow.canvas.update();
    mainWindow.canvas.fitScene();

    //Visualize mesh
    ui->meshRadio->setChecked(true);
    initializeVisualizationSlider();

    updateUI();
}

void FAFManager::on_saliencyFindDetailsButton_clicked(){
    //Find details by saliency
    findDetails();

    //Update canvas and fit the scene
    mainWindow.canvas.update();
    mainWindow.canvas.fitScene();

    //Visualize mesh
    ui->meshRadio->setChecked(true);
    initializeVisualizationSlider();

    updateUI();
}

void FAFManager::on_smoothingButton_clicked() {
    //Get optimal mesh orientation
    smoothing();

    //Update canvas and fit the scene
    mainWindow.canvas.update();
    mainWindow.canvas.fitScene();

    //Visualize mesh
    ui->meshRadio->setChecked(true);
    initializeVisualizationSlider();

    updateUI();
}

void FAFManager::on_optimalOrientationButton_clicked() {
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

void FAFManager::on_selectExtremesButton_clicked()
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

void FAFManager::on_checkVisibilityButton_clicked()
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

void FAFManager::on_getAssociationButton_clicked()
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

void FAFManager::on_optimizationButton_clicked()
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

void FAFManager::on_smoothLinesButton_clicked()
{
    //Optimization
    smoothLines();

    //Update canvas and fit the scene
    mainWindow.canvas.update();
    mainWindow.canvas.fitScene();

    //Visualize association
    ui->associationRadio->setChecked(true);
    initializeVisualizationSlider();

    updateUI();
}

void FAFManager::on_restoreFrequenciesButton_clicked() {
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

void FAFManager::on_cutComponentsButton_clicked() {
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


void FAFManager::on_extractResultsButton_clicked() {
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


void FAFManager::on_centerOnOriginButton_clicked() {
    if (data.isMeshLoaded){
        //Translation of the mesh
        data.mesh.translate(-data.mesh.boundingBox().center());

        //Update canvas and fit the scene
        mainWindow.canvas.update();
        mainWindow.canvas.fitScene();
    }
}

void FAFManager::on_plusXButton_clicked() {
    if (data.isMeshLoaded){
        //Translation of the mesh
        data.mesh.translate(cg3::Point3d(ui->stepSpinBox->value(), 0, 0));

        //Update canvas and fit the scene
        mainWindow.canvas.update();
        mainWindow.canvas.fitScene();
    }
}

void FAFManager::on_minusXButton_clicked() {
    if (data.isMeshLoaded){
        //Translation of the mesh
        data.mesh.translate(cg3::Point3d(-ui->stepSpinBox->value(), 0, 0));

        //Update canvas and fit the scene
        mainWindow.canvas.update();
        mainWindow.canvas.fitScene();
    }
}

void FAFManager::on_plusYButton_clicked() {
    if (data.isMeshLoaded){
        //Translation of the mesh
        data.mesh.translate(cg3::Point3d(0, ui->stepSpinBox->value(), 0));

        //Update canvas and fit the scene
        mainWindow.canvas.update();
        mainWindow.canvas.fitScene();
    }
}

void FAFManager::on_minusYButton_clicked() {
    if (data.isMeshLoaded){
        //Translation of the mesh
        data.mesh.translate(cg3::Point3d(0, -ui->stepSpinBox->value(), 0));

        //Update canvas and fit the scene
        mainWindow.canvas.update();
        mainWindow.canvas.fitScene();
    }
}

void FAFManager::on_plusZButton_clicked() {
    if (data.isMeshLoaded){
        //Translation of the mesh
        data.mesh.translate(cg3::Point3d(0, 0, ui->stepSpinBox->value()));

        //Update canvas and fit the scene
        mainWindow.canvas.update();
        mainWindow.canvas.fitScene();
    }
}

void FAFManager::on_minusZButton_clicked() {
    if (data.isMeshLoaded){
        //Translation of the mesh
        data.mesh.translate(cg3::Point3d(0, 0, -ui->stepSpinBox->value()));

        //Update canvas and fit the scene
        mainWindow.canvas.update();
        mainWindow.canvas.fitScene();
    }
}

void FAFManager::on_rotateButton_clicked() {
    if (data.isMeshLoaded){
        //Rotation of the mesh
        cg3::Vec3d axis(ui->axisXSpinBox->value(), ui->axisYSpinBox->value(), ui->axisZSpinBox->value());
        double angle = ui->angleSpinBox->value() * M_PI/180;

        Eigen::Matrix3d m;
        cg3::rotationMatrix(axis, angle, m);

        data.mesh.rotate(m);

        //Update canvas and fit the scene
        mainWindow.canvas.update();
        mainWindow.canvas.fitScene();
    }
}

void FAFManager::on_scaleButton_clicked() {
    if (data.isMeshLoaded){
        //Scale the mesh
        cg3::Vec3d scaleFactor(ui->scaleXSpinBox->value(), ui->scaleYSpinBox->value(), ui->scaleZSpinBox->value());

        data.mesh.scale(scaleFactor);

        //Update canvas and fit the scene
        mainWindow.canvas.update();
        mainWindow.canvas.fitScene();
    }
}

void FAFManager::on_inverseScaleButton_clicked() {
    if (data.isMeshLoaded){
        //Scale the mesh
        cg3::Vec3d scaleFactor(1.0/ui->scaleXSpinBox->value(), 1.0/ui->scaleYSpinBox->value(), 1.0/ui->scaleZSpinBox->value());

        data.mesh.scale(scaleFactor);

        //Update canvas and fit the scene
        mainWindow.canvas.update();
        mainWindow.canvas.fitScene();
    }
}




/* ----- UI SLOTS VISUALIZATION ------ */

void FAFManager::on_meshRadio_clicked() {
    initializeVisualizationSlider();
}

void FAFManager::on_extremesRadio_clicked() {
    initializeVisualizationSlider();
}

void FAFManager::on_visibilityRadio_clicked() {
    initializeVisualizationSlider();
}

void FAFManager::on_associationRadio_clicked() {
    initializeVisualizationSlider();
}

void FAFManager::on_resultsRadio_clicked() {
    initializeVisualizationSlider();
}

void FAFManager::on_showNonVisibleCheck_clicked()
{
    initializeVisualizationSlider();
}
void FAFManager::on_resetCameraButton_clicked() {
    //Reset camera direction
    resetCameraDirection();
}

void FAFManager::on_visualizationSlider_valueChanged(int value) {
    CG3_SUPPRESS_WARNING(value);
    updateVisualization();

    unsigned int sliderValue = (unsigned int) ui->visualizationSlider->value();

    if (ui->moveCameraCheckBox->isChecked()) {
        if (ui->extremesRadio->isChecked()) {
            cg3::Vec3d xAxis(1,0,0);
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
                cg3::Vec3d currentDirection = data.directions[sliderValue - 1];
                setCameraDirection(-currentDirection);
            }
            else {
                resetCameraDirection();
            }
        }
        else if (ui->associationRadio->isChecked()) {
            if (sliderValue > 0) {
                unsigned int currentIndex = data.targetDirections[sliderValue - 1];
                cg3::Vec3d currentDirection = data.directions[currentIndex];
                setCameraDirection(-currentDirection);
            }
            else {
                resetCameraDirection();
            }
        }
        else if (ui->resultsRadio->isChecked()) {
            if (ui->extractResultsRotateCheckBox->isChecked()) {
                cg3::Vec3d zAxis(0,0,1);
                setCameraDirection(-zAxis);
            }
            else {
                unsigned int currentIndex = data.targetDirections[sliderValue];
                cg3::Vec3d currentDirection = data.directions[currentIndex];
                setCameraDirection(-currentDirection);
            }

            for (cg3::DrawableEigenMesh& res : drawableResults) {
                mainWindow.setDrawableObjectVisibility(&res, false);
            }
            mainWindow.setDrawableObjectVisibility(&drawableResults[sliderValue], true);
        }
    }
}

void FAFManager::meshPainted()
{
    colorizeDetailMesh();
}

void FAFManager::facePicked(const cg3::PickableObject* obj, unsigned int f)
{
    if (obj == static_cast<cg3::PickableObject*>(&drawableDetailMesh)) {
        if (data.isSaliencyComputed && !data.isMeshSmoothed) {
            const double brushSize = static_cast<float>(ui->paintingSizeSlider->value()) * (drawableDetailMesh.boundingBox().diag() / 100.f);
            const double fillValue = static_cast<float>(ui->paintingFillValueSpinBox->value());

            std::vector<unsigned int> sourceVertices(3);
            sourceVertices[0] = drawableDetailMesh.face(f).x();
            sourceVertices[1] = drawableDetailMesh.face(f).y();
            sourceVertices[2] = drawableDetailMesh.face(f).z();

            std::vector<double> vertexGeodesics;

//            cg3::libigl::exactGeodesics(
//                        drawableDetailMesh,
//                        sourceVertices,
//                        vertexGeodesics);

            vertexGeodesics = cg3::libigl::heatGeodesics(
                        detailMeshGeodesicsData,
                        sourceVertices);

            for(uint fId = 0; fId < drawableDetailMesh.numberFaces(); ++fId) {
                double dist = vertexGeodesics[drawableDetailMesh.face(fId).x()];
                dist = std::max(dist, vertexGeodesics[drawableDetailMesh.face(fId).y()]);
                dist = std::max(dist, vertexGeodesics[drawableDetailMesh.face(fId).z()]);

                if (fId == f || dist <= brushSize) {
                    if (ui->paintingFillRadioBox->isChecked()) {
                        data.faceSaliency[fId] = fillValue;
                    }
                    else if (ui->paintingFreeRadioBox->isChecked()) {
                        data.faceSaliency[fId] = 0.0;
                    }
                    else if (ui->paintingResetRadioBox->isChecked()) {
                        data.faceSaliency[fId] = (
                            data.saliency[drawableDetailMesh.face(fId).x()] +
                            data.saliency[drawableDetailMesh.face(fId).y()] +
                            data.saliency[drawableDetailMesh.face(fId).z()]
                        ) / 3;
                    }
                }
            }

            colorizeDetailMesh();
        }
    }
}
