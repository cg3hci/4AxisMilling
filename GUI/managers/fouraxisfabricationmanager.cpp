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

    ui->computeEntireAlgorithmButton->setEnabled(!isMeshOriented);

    //Optimal orientation
    ui->optimalOrientationButton->setEnabled(!isMeshOriented);
    ui->nOrientationLabel->setEnabled(!isMeshOriented);
    ui->nOrientationSpinBox->setEnabled(!isMeshOriented);
    ui->deterministicCheckBox->setEnabled(!isMeshOriented);

    //Select extremes
    ui->selectExtremesButton->setEnabled(!areExtremesSelected);
    ui->fixExtremeAssociationCheckBox->setEnabled(!isVisibilityChecked);

    //Check visibility
    ui->checkVisibilityButton->setEnabled(!isVisibilityChecked);
    ui->nDirectionsLabel->setEnabled(!isVisibilityChecked);
    ui->nDirectionsSpinBox->setEnabled(!isVisibilityChecked);
    ui->visibilityMethodFrame->setEnabled(!isVisibilityChecked);

    //Get the target directions
    ui->targetDirectionsButton->setEnabled(!areTargetDirectionsFound);
    ui->setCoverageCheckBox->setEnabled(!areTargetDirectionsFound);

    //Get association
    ui->getAssociationButton->setEnabled(!isAssociationComputed);
    ui->compactnessLabel->setEnabled(!isAssociationComputed);
    ui->compactnessSpinBox->setEnabled(!isAssociationComputed);
    ui->limitAngleSpinBox->setEnabled(!isAssociationComputed);
    ui->limitAngleLabel->setEnabled(!isAssociationComputed);

    //Restore frequencies
    ui->restoreFrequenciesButton->setEnabled(!areFrequenciesRestored);
    ui->nIterationsLabel->setEnabled(!areFrequenciesRestored);
    ui->nIterationsSpinBox->setEnabled(!areFrequenciesRestored);
    ui->updateAssociationCheckBox->setEnabled(!areFrequenciesRestored);

    //Cut components
    ui->cutComponentsButton->setEnabled(!areComponentsCut);

    //Extract surfaces
    ui->extractSurfacesButton->setEnabled(!areSurfacesExtracted);


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
    areFrequenciesRestored = false;
    areComponentsCut = false;
    areSurfacesExtracted = false;

    data.clear();

    originalMesh.clear();
    smoothedMesh.clear();
}




/* ----- COMPUTING METHODS ------ */


/**
 * @brief Compute entire algorithm
 */
void FourAxisFabricationManager::computeEntireAlgorithm() {
    if (!isMeshOriented) {
        std::cout << std::endl << "#######################################################################" << std::endl << std::endl;

        //Get UI data
        unsigned int nOrientation = (unsigned int) ui->nOrientationSpinBox->value();
        bool deterministic = ui->deterministicCheckBox->isChecked();
        unsigned int nDirections = (unsigned int) ui->nDirectionsSpinBox->value();
        bool fixExtremeAssociation = ui->fixExtremeAssociationCheckBox->isChecked();
        FourAxisFabrication::CheckMode checkMode = (ui->rayShootingRadio->isChecked() ?
                FourAxisFabrication::RAYSHOOTING :
                FourAxisFabrication::PROJECTION);
        bool setCoverageFlag = ui->setCoverageCheckBox->isChecked();
        double compactness = ui->compactnessSpinBox->value();
        double limitAngle = ui->limitAngleSpinBox->value() / 180.0 * M_PI;
        unsigned int nIterations = (unsigned int) ui->nIterationsSpinBox->value();
        bool updateAssociationAfterFrequencyRestoring = ui->updateAssociationCheckBox->isChecked();

        cg3::Timer t("Entire algorithm");

        //Get optimal mesh orientation
        FourAxisFabrication::computeEntireAlgorithm(
                    originalMesh,
                    smoothedMesh,
                    nOrientation,
                    deterministic,
                    nDirections,
                    fixExtremeAssociation,
                    setCoverageFlag,
                    compactness,
                    limitAngle,
                    nIterations,
                    updateAssociationAfterFrequencyRestoring,
                    data,
                    checkMode);


        t.stopAndPrint();

        isMeshOriented = true;
        areExtremesSelected = true;
        isVisibilityChecked = true;
        areTargetDirectionsFound = true;
        isAssociationComputed = true;
        areFrequenciesRestored = true;
        areComponentsCut = true;
        areSurfacesExtracted = true;

        updateDrawableMeshes();
        addDrawableCutComponents();
        addDrawableSurfaces();
    }
}

/**
 * @brief Compute optimal orientation
 */
void FourAxisFabricationManager::optimalOrientation() {
    if (!isMeshOriented) {
        std::cout << std::endl << "#######################################################################" << std::endl << std::endl;

        //Get UI data
        unsigned int nOrientation = (unsigned int) ui->nOrientationSpinBox->value();
        bool deterministic = ui->deterministicCheckBox->isChecked();

        cg3::Timer t("Optimal orientation");

        //Get optimal mesh orientation
        FourAxisFabrication::rotateToOptimalOrientation(
                    originalMesh,
                    smoothedMesh,
                    nOrientation,
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

        cg3::Timer t("Select extremes");

        //Get extremes on x-axis to be selected
        FourAxisFabrication::selectExtremesOnXAxis(smoothedMesh, data);

        t.stopAndPrint();

        areExtremesSelected = true;
    }
}

/**
 * @brief Check visibility from various directions
 */
void FourAxisFabricationManager::checkVisibility() {
    if (!isVisibilityChecked) {
        optimalOrientation();
        selectExtremes();

        //Get UI data
        unsigned int nDirections = (unsigned int) ui->nDirectionsSpinBox->value();
        bool fixExtremeAssociation = ui->fixExtremeAssociationCheckBox->isChecked();
        FourAxisFabrication::CheckMode checkMode = (ui->rayShootingRadio->isChecked() ?
                FourAxisFabrication::RAYSHOOTING :
                FourAxisFabrication::PROJECTION);


        cg3::Timer t("Visibility check");

        //Visibility check
        FourAxisFabrication::getVisibility(
                    smoothedMesh,
                    nDirections,
                    fixExtremeAssociation,
                    data,
                    checkMode);


        t.stopAndPrint();

        isVisibilityChecked = true;
    }
}

/**
 * @brief Get the target directions
 */
void FourAxisFabricationManager::getTargetDirections() {
    if (!areTargetDirectionsFound) {
        optimalOrientation();
        selectExtremes();
        checkVisibility();

        //Get UI data
        bool setCoverageFlag = ui->setCoverageCheckBox->isChecked();

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
 * @brief Get optimized association
 */
void FourAxisFabricationManager::getAssociation() {
    if (!isAssociationComputed) {
        optimalOrientation();
        selectExtremes();
        checkVisibility();
        getTargetDirections();

        //Get UI data
        double compactness = ui->compactnessSpinBox->value();
        double limitAngle = ui->limitAngleSpinBox->value() / 180.0 * M_PI;

        cg3::Timer t("Get association");

        //Get association
        FourAxisFabrication::getOptimizedAssociation(
                    smoothedMesh,
                    compactness,
                    limitAngle,
                    data);

        t.stopAndPrint();

        isAssociationComputed = true;
    }
}

/**
 * @brief Restore frequencies
 */
void FourAxisFabricationManager::restoreFrequencies() {
    if (!areFrequenciesRestored) {
        optimalOrientation();
        selectExtremes();
        checkVisibility();
        getTargetDirections();
        getAssociation();

        //Get UI data
        unsigned int nIterations = (unsigned int) ui->nIterationsSpinBox->value();
        FourAxisFabrication::CheckMode checkMode = (ui->rayShootingRadio->isChecked() ?
                FourAxisFabrication::RAYSHOOTING :
                FourAxisFabrication::PROJECTION);
        bool updateAssociationAfterFrequencyRestoring = ui->updateAssociationCheckBox->isChecked();

        cg3::Timer t("Restore frequencies");

        //Restore frequencies
        FourAxisFabrication::restoreFrequencies(originalMesh, data, nIterations, smoothedMesh);

        t.stopAndPrint();


        if (updateAssociationAfterFrequencyRestoring) {
            cg3::Timer tCheck("Update association after frequencies have been restored");

            bool isValidAssociation =
                    FourAxisFabrication::updateAssociationIfNotVisible(smoothedMesh, data, checkMode);

            tCheck.stopAndPrint();

            if (!isValidAssociation) {
                QMessageBox::warning(this, "Warning", "Association is not valid after the frequencies have been restored!");
            }
        }


        areFrequenciesRestored = true;

        updateDrawableMeshes();
    }
}


/**
 * @brief Cut components
 */
void FourAxisFabricationManager::cutComponents() {
    if (!areComponentsCut) {
        optimalOrientation();
        selectExtremes();
        checkVisibility();
        getTargetDirections();
        getAssociation();
        restoreFrequencies();


        cg3::Timer t("Cut components");

        //Cut components
        FourAxisFabrication::cutComponents(smoothedMesh, data);

        t.stopAndPrint();

        areComponentsCut = true;

        addDrawableCutComponents();
    }
}

/**
 * @brief Extract surfaces
 */
void FourAxisFabricationManager::extractSurfaces() {
    if (!areSurfacesExtracted) {
        optimalOrientation();
        selectExtremes();
        checkVisibility();
        getTargetDirections();
        getAssociation();
        restoreFrequencies();
        cutComponents();


        cg3::Timer t("Extract surfaces");

        //Extract surfaces
        FourAxisFabrication::extractSurfaces(data);

        t.stopAndPrint();

        areSurfacesExtracted = true;

        addDrawableSurfaces();
    }
}




/* ----- VISUALIZATION METHODS ------ */

/**
 * @brief Update drawable meshes
 */
void FourAxisFabricationManager::updateDrawableMeshes() {
    //Create drawable meshes (already in the canvas)
    drawableOriginalMesh = cg3::DrawableEigenMesh(originalMesh);
    drawableSmoothedMesh = cg3::DrawableEigenMesh(smoothedMesh);

    mainWindow.refreshDrawableObject(&drawableOriginalMesh);
    mainWindow.refreshDrawableObject(&drawableSmoothedMesh);
}

/**
 * @brief Add drawable cut components to the canvas
 */
void FourAxisFabricationManager::addDrawableCutComponents() {
    //Hide smoothed mesh
    mainWindow.setDrawableObjectVisibility(&drawableSmoothedMesh, false);

    //Create drawable meshes
    drawableMinComponent = cg3::DrawableEigenMesh(data.minComponent);
    drawableMaxComponent = cg3::DrawableEigenMesh(data.maxComponent);
    drawableFourAxisComponent = cg3::DrawableEigenMesh(data.fourAxisComponent);

    //Push in the canvas
    mainWindow.pushDrawableObject(&drawableMinComponent, "Min component");
    mainWindow.pushDrawableObject(&drawableMaxComponent, "Max component");
    mainWindow.pushDrawableObject(&drawableFourAxisComponent, "4-axis component");
}

/**
 * @brief Add drawable surfaces
 */
void FourAxisFabricationManager::addDrawableSurfaces() {
    //Hide the cut components
    mainWindow.setDrawableObjectVisibility(&drawableMinComponent, false);
    mainWindow.setDrawableObjectVisibility(&drawableMaxComponent, false);
    mainWindow.setDrawableObjectVisibility(&drawableFourAxisComponent, false);

    //TODO CONTAINER
    //Draw components
//    drawableComponentsContainer.clear();
//    drawableComponents.resize(data.surfaces.size());
//    for (size_t i = 0; i < data.surfaces.size(); i++) {
//        drawableComponents[i] = cg3::DrawableEigenMesh(data.surfaces[i]);
//        drawableComponentsContainer.pushBack(&drawableComponents[i], "Surface " + std::to_string(i));
//    }
//    mainWindow.pushDrawableObject(&drawableComponentsContainer, "Surfaces");

    drawableComponents.resize(data.surfaces.size());
    for (size_t i = 0; i < data.surfaces.size(); i++) {
        drawableComponents[i] = cg3::DrawableEigenMesh(data.surfaces[i]);
        mainWindow.pushDrawableObject(&drawableComponents[i], "Surface " + std::to_string(i));
    }
}



/**
 * @brief Delte all drawable objects from the canvas
 */
void FourAxisFabricationManager::deleteDrawableObjects() {
    if (isMeshLoaded) {
        //Delete meshes
        mainWindow.deleteDrawableObject(&drawableOriginalMesh);
        mainWindow.deleteDrawableObject(&drawableSmoothedMesh);

        //Delete cut components
        if (areComponentsCut) {
            mainWindow.deleteDrawableObject(&drawableMinComponent);
            mainWindow.deleteDrawableObject(&drawableMaxComponent);
            mainWindow.deleteDrawableObject(&drawableFourAxisComponent);

            //Delete surfaces
            if (areSurfacesExtracted) {
                //TODO CONTAINER
//                mainWindow.deleteDrawableObject(&drawableComponentsContainer);

                for (cg3::DrawableEigenMesh& drawableMesh : drawableComponents) {
                    mainWindow.deleteDrawableObject(&drawableMesh);
                }
            }
        }
    }    

    drawableOriginalMesh.clear();
    drawableSmoothedMesh.clear();

    drawableMinComponent.clear();
    drawableMaxComponent.clear();
    drawableFourAxisComponent.clear();

    drawableComponents.clear();

    //TODO CONTAINER
//    drawableComponentsContainer.clear();
}



/**
 * @brief Reset camera pointing to the z-axis direction
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
void FourAxisFabricationManager::setCameraDirection(cg3::Vec3 dir) {
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
    }
    else if (ui->extremesRadio->isChecked()) {
        ui->visualizationSlider->setMinimum(0);
        ui->visualizationSlider->setMaximum(2);
        ui->visualizationSlider->setValue(0);
    }
    else if (ui->visibilityRadio->isChecked()) {
        ui->visualizationSlider->setMinimum(0);
        ui->visualizationSlider->setMaximum(data.visibility.getSizeX());
        ui->visualizationSlider->setValue(0);
    }
    else if (ui->targetDirectionsRadio->isChecked() || ui->associationRadio->isChecked()) {
        ui->visualizationSlider->setMinimum(0);
        ui->visualizationSlider->setMaximum(data.targetDirections.size());
        ui->visualizationSlider->setValue(0);
    }

    updateVisualization();
}

/**
 * @brief Colorize mesh depending on which radio button you have chosen
 */
void FourAxisFabricationManager::updateVisualization() {
    if (ui->meshRadio->isChecked()) {
        visualizeMesh();
    }
    else if (ui->extremesRadio->isChecked()) {
        visualizeExtremes();
    }
    else if (ui->visibilityRadio->isChecked()) {
        visualizeVisibility();
    }
    else if (ui->targetDirectionsRadio->isChecked()) {
        visualizeTargetDirections();
    }
    else if (ui->associationRadio->isChecked()) {
        visualizeAssociation();
    }


    //Update canvas
    mainWindow.canvas.update();

    updateUI();
}

/**
 * @brief Colorize mesh to the default color
 */
void FourAxisFabricationManager::visualizeMesh() {
    //Default color
    drawableSmoothedMesh.setFaceColor(cg3::Color(128,128,128));
    drawableMinComponent.setFaceColor(cg3::Color(128,128,128));
    drawableMaxComponent.setFaceColor(cg3::Color(128,128,128));
    drawableFourAxisComponent.setFaceColor(cg3::Color(128,128,128));

    ui->descriptionLabel->setText(""); //Empty description text
}


/**
 * @brief Colorize the min and max extremes
 */
void FourAxisFabricationManager::visualizeExtremes() {
    //Default color
    drawableSmoothedMesh.setFaceColor(cg3::Color(128,128,128));


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
void FourAxisFabricationManager::visualizeVisibility() {
    unsigned int sliderValue = (unsigned int) ui->visualizationSlider->value();

    //Set default color
    drawableSmoothedMesh.setFaceColor(cg3::Color(128,128,128));

    std::stringstream ss;

    //Color the not visible
    if (sliderValue == 0) {
        for (unsigned int faceId : data.nonVisibleFaces) {
            drawableSmoothedMesh.setFaceColor(cg3::Color(0,0,0), faceId);
        }

        ss << "Visibility: " << data.directions.size() << " directions. ";
        if (data.nonVisibleFaces.size() > 0) {
            ss << data.nonVisibleFaces.size() << " non-visible triangles (black)";
        }
        else {
            ss << "All the triangles are visible";
        }
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
        for (unsigned int j = 0; j < data.visibility.getSizeY(); j++) {
            if (data.visibility(chosenDirectionIndex, j) == 1) {
                //Set the color
                drawableSmoothedMesh.setFaceColor(color, j);
            }
        }

        //Description
        ss << "Direction " << data.directions[chosenDirectionIndex];
    }

    //Update description label
    std::string description = ss.str();
    ui->descriptionLabel->setText(QString::fromStdString(description));
}

/**
 * @brief Colorize face visibility from the target directions
 */
void FourAxisFabricationManager::visualizeTargetDirections() {
    unsigned int sliderValue = (unsigned int) ui->visualizationSlider->value();

    //Set default color
    drawableSmoothedMesh.setFaceColor(cg3::Color(128,128,128));

    std::stringstream ss;

    //Color the not visible
    if (sliderValue == 0) {
        for (unsigned int faceId : data.nonVisibleFaces) {
            drawableSmoothedMesh.setFaceColor(cg3::Color(0,0,0), faceId);
        }

        //Description
        ss << "Target directions: " << data.targetDirections.size() << " directions. ";
        if (data.nonVisibleFaces.size() > 0) {
            ss << data.nonVisibleFaces.size() << " non-visible triangles (black)";
        }
        else {
            ss << "All the triangles are visible";
        }
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
        for (unsigned int j = 0; j < data.visibility.getSizeY(); j++) {
            if (data.visibility(chosenDirectionIndex, j) == 1) {
                //Set the color
                drawableSmoothedMesh.setFaceColor(color, j);
            }
        }

        //Description
        ss << "Direction " << data.directions[chosenDirectionIndex];
    }

    //Update description label
    std::string description = ss.str();
    ui->descriptionLabel->setText(QString::fromStdString(description));
}



/**
 * @brief Colorize association
 */
void FourAxisFabricationManager::visualizeAssociation() {
    std::stringstream ss;    

    //Coloring drawable mesh
    visualizeAssociation(drawableSmoothedMesh, data.association, data.targetDirections);

    //Coloring cut components
    if (areComponentsCut) {
        visualizeAssociation(drawableMinComponent, data.minComponentAssociation, data.targetDirections);
        visualizeAssociation(drawableMaxComponent, data.maxComponentAssociation, data.targetDirections);
        visualizeAssociation(drawableFourAxisComponent, data.fourAxisComponentAssociation, data.targetDirections);
    }

    //Get UI data
    unsigned int sliderValue = (unsigned int) ui->visualizationSlider->value();

    //Description
    if (sliderValue > 0) {
        //Get index of the current chosen direction
        unsigned int chosenDirectionIndex = data.targetDirections[sliderValue - 1];

        ss << "Direction " << data.directions[chosenDirectionIndex];
    }
    else {
        ss << "Optimization: " << data.targetDirections.size() << " directions. ";
        if (data.nonVisibleFaces.size() > 0) {
            ss << data.nonVisibleFaces.size() << " non-visible triangles (black)";
        }
        else {
            ss << "All the triangles are visible";
        }
    }


    //Update description label
    std::string description = ss.str();
    ui->descriptionLabel->setText(QString::fromStdString(description));
}


/**
 * @brief Colorize association for a given mesh
 * @param drawableMesh Drawable mesh to be colorized
 * @param association Association of the target directions
 * @param targetDirections Target directions
 */
void FourAxisFabricationManager::visualizeAssociation(
        cg3::DrawableEigenMesh& drawableMesh,
        const std::vector<int>& association,
        const std::vector<unsigned int>& targetDirections)
{
    unsigned int sliderValue = (unsigned int) ui->visualizationSlider->value();

    //Variables for colors
    cg3::Color color;
    int subd = 255 / (data.targetDirections.size()-1);


    //For each face of the drawable smoothed mesh
    for (unsigned int faceId = 0; faceId < drawableMesh.getNumberFaces(); faceId++) {
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
        else {
            //Black color for non-visible faces
            drawableMesh.setFaceColor(cg3::Color(0,0,0), faceId);
        }
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
            isMeshLoaded = originalMesh.readFromObj(meshFile);

            //If the mesh has been successfully loaded
            if (isMeshLoaded){
                std::string rawname, ext;
                cg3::separateExtensionFromFilename(meshFile, rawname, ext);

                //Find smoothed mesh in the path
                smoothedFile = rawname + "_smooth" + ext;
                isMeshLoaded = smoothedMesh.readFromObj(smoothedFile);

                //If a smoothed mesh has not been found
                if (!isMeshLoaded){
                    //Get loading dialog
                    smoothedFile = loaderSaverObj.loadDialog("Load smoothed mesh");
                    if (smoothedFile != ""){
                        isMeshLoaded = smoothedMesh.readFromObj(smoothedFile);
                    }
                }

                //If a smoothed mesh has been found
                if (isMeshLoaded) {
                    updateDrawableMeshes();

                    //Add meshes to the canvas, hiding the original one
                    std::string meshName = meshFile.substr(meshFile.find_last_of("/") + 1);
                    mainWindow.pushDrawableObject(&drawableOriginalMesh, meshName);
                    mainWindow.pushDrawableObject(&drawableSmoothedMesh, "Smoothed mesh");
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
                                 "Number of faces: " << originalMesh.getNumberFaces() << std::endl;
                    std::cout << "Smoothed mesh file: \"" << loadedSmoothedMeshFile << "\"" << std::endl <<
                                 "Number of faces: " << smoothedMesh.getNumberFaces() << std::endl;
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
                originalMesh.readFromObj(loadedMeshFile) &
                smoothedMesh.readFromObj(loadedSmoothedMeshFile);

        //If the meshes have been successfully loaded
        if (isMeshLoaded){
            updateDrawableMeshes();

            //Add meshes to the canvas, hiding the original one
            std::string meshName = loadedMeshFile.substr(loadedMeshFile.find_last_of("/") + 1);
            mainWindow.pushDrawableObject(&drawableOriginalMesh, meshName);
            mainWindow.pushDrawableObject(&drawableSmoothedMesh, "Smoothed mesh");
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

        if (areComponentsCut) {
            data.minComponent.setVertexColor(128,128,128);
            data.maxComponent.setVertexColor(128,128,128);
            data.fourAxisComponent.setVertexColor(128,128,128);

            data.minComponent.saveOnObj(rawname + "_min.obj");
            data.maxComponent.saveOnObj(rawname + "_max.obj");
            data.fourAxisComponent.saveOnObj(rawname + "_fouraxis.obj");

            if (areSurfacesExtracted) {
                for (size_t i = 0; i < data.surfaces.size(); i++) {
                    cg3::EigenMesh& mesh = data.surfaces[i];
                    mesh.setVertexColor(128,128,128);
                    mesh.saveOnObj(rawname + "_component_" + std::to_string(i) + ".obj");

                    cg3::EigenMesh copyMesh = mesh;

                    Eigen::Matrix3d rotationMatrix;
                    cg3::Vec3 xAxis(1,0,0);
                    cg3::Vec3 yAxis(0,1,0);

                    if (i < data.surfaces.size() - 2) {
                        unsigned int label = data.surfacesAssociation[i];
                        double angle = data.angles[label];
                        cg3::getRotationMatrix(xAxis, angle, rotationMatrix);
                    }
                    //Min
                    else if (i == data.surfaces.size() - 2) {
                        cg3::getRotationMatrix(yAxis, M_PI/2, rotationMatrix);

                        //Center mesh
                        copyMesh.translate(-copyMesh.getBoundingBox().center());
                    }
                    //Max
                    else {
                        assert(i == data.surfaces.size() - 1);
                        cg3::getRotationMatrix(yAxis, -M_PI/2, rotationMatrix);

                        //Center mesh
                        copyMesh.translate(-copyMesh.getBoundingBox().center());
                    }

                    //Rotate mesh
                    copyMesh.rotate(rotationMatrix);

                    copyMesh.saveOnObj(rawname + "_rotated_component_" + std::to_string(i) + ".obj");

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
                }
            }
        }
    }
}


/* ----- UI SLOTS FOUR AXIS FABRICATION ------ */


void FourAxisFabricationManager::on_computeEntireAlgorithmButton_clicked() {
    if (isMeshLoaded){
        //Compute the entire algorithm
        computeEntireAlgorithm();

        //Visualize association
        ui->associationRadio->setChecked(true);
        initializeVisualizationSlider();

        //Update canvas and fit the scene
        mainWindow.canvas.update();
        mainWindow.canvas.fitScene();

        updateUI();
    }
}


void FourAxisFabricationManager::on_optimalOrientationButton_clicked() {
    //Get optimal mesh orientation
    optimalOrientation();

    //Visualize mesh
    ui->meshRadio->setChecked(true);
    initializeVisualizationSlider();

    //Update canvas and fit the scene
    mainWindow.canvas.update();
    mainWindow.canvas.fitScene();

    updateUI();
}

void FourAxisFabricationManager::on_selectExtremesButton_clicked()
{
    //Get extremes on x-axis to be selected
    selectExtremes();

    //Visualize extremes
    ui->extremesRadio->setChecked(true);
    initializeVisualizationSlider();

    //Update canvas and fit the scene
    mainWindow.canvas.update();
    mainWindow.canvas.fitScene();

    updateUI();
}

void FourAxisFabricationManager::on_checkVisibilityButton_clicked()
{
    //Check visibility by the chosen directions
    checkVisibility();

    //Visualize visibility
    ui->visibilityRadio->setChecked(true);
    initializeVisualizationSlider();

    //Update canvas and fit the scene
    mainWindow.canvas.update();
    mainWindow.canvas.fitScene();

    updateUI();
}


void FourAxisFabricationManager::on_targetDirectionsButton_clicked()
{
    //Get target milling directions
    getTargetDirections();

    //Visualize target directions
    ui->targetDirectionsRadio->setChecked(true);
    initializeVisualizationSlider();

    //Update canvas and fit the scene
    mainWindow.canvas.update();
    mainWindow.canvas.fitScene();

    updateUI();
}

void FourAxisFabricationManager::on_getAssociationButton_clicked()
{
    //Check visibility by the chosen directions
    getAssociation();

    //Visualize association
    ui->associationRadio->setChecked(true);
    initializeVisualizationSlider();

    //Update canvas and fit the scene
    mainWindow.canvas.update();
    mainWindow.canvas.fitScene();

    updateUI();
}

void FourAxisFabricationManager::on_restoreFrequenciesButton_clicked() {
    //Check visibility by the chosen directions
    restoreFrequencies();

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

    //Visualize association
    ui->associationRadio->setChecked(true);
    initializeVisualizationSlider();

    //Update canvas and fit the scene
    mainWindow.canvas.update();
    mainWindow.canvas.fitScene();

    updateUI();
}

void FourAxisFabricationManager::on_extractSurfacesButton_clicked() {
    //Extract surfaces
    extractSurfaces();

    //Visualize association
    ui->associationRadio->setChecked(true);
    initializeVisualizationSlider();

    //Update canvas and fit the scene
    mainWindow.canvas.update();
    mainWindow.canvas.fitScene();

    updateUI();
}


/* ----- UI SLOTS TRANSFORMATIONS ------ */


void FourAxisFabricationManager::on_centerOnOriginButton_clicked() {
    if (isMeshLoaded){
        //Translation of the mesh
        originalMesh.translate(-smoothedMesh.getBoundingBox().center());
        smoothedMesh.translate(-smoothedMesh.getBoundingBox().center());

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
        cg3::getRotationMatrix(axis, angle, m);

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

void FourAxisFabricationManager::on_resetCameraButton_clicked() {
    resetCameraDirection();
}
