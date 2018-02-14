/**
 * @author Stefano Nuvoli
 * @author Alessandro Muntoni
 */
#include "fouraxisfabricationmanager.h"
#include "ui_fouraxisfabricationmanager.h"

#include <sstream>
#include <string>

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
    isMeshLoaded = false;
    isMeshOriented = false;
    areExtremesCut = false;
    isVisibilityChecked = false;
    areTargetDirectionsFound = false;
    isAssociationComputed = false;

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


    // ----- Four axis fabrication -----
    ui->fourAxisFabricationGroup->setEnabled(isMeshLoaded);

    ui->computeEntireAlgorithmButton->setEnabled(!isMeshOriented);

    //Optimal orientation
    ui->optimalOrientationButton->setEnabled(!isMeshOriented);
    ui->nOrientationLabel->setEnabled(!isMeshOriented);
    ui->nOrientationSpinBox->setEnabled(!isMeshOriented);
    ui->deterministicCheckBox->setEnabled(!isMeshOriented);

    //Cut extremes
    ui->cutExtremesButton->setEnabled(isMeshOriented && !areExtremesCut);
    ui->fixExtremeAssociationCheckBox->setEnabled(!isVisibilityChecked);

    //Check visibility
    ui->checkVisibilityButton->setEnabled(areExtremesCut && !isVisibilityChecked);
    ui->nDirectionsLabel->setEnabled(!isVisibilityChecked);
    ui->nDirectionsSpinBox->setEnabled(!isVisibilityChecked);
    ui->visibilityMethodFrame->setEnabled(!isVisibilityChecked);

    //Get the target directions
    ui->targetDirectionsButton->setEnabled(isVisibilityChecked && !areTargetDirectionsFound);
    ui->setCoverageCheckBox->setEnabled(!areTargetDirectionsFound);

    //Graph cut    
    ui->getAssociationButton->setEnabled(areTargetDirectionsFound && !isAssociationComputed);


    // ----- Visualization -----
    ui->visualizationGroup->setEnabled(isMeshLoaded);

    //Radio
    ui->meshRadio->setEnabled(isMeshLoaded);
    ui->extremesRadio->setEnabled(areExtremesCut);
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
    areExtremesCut = false;
    isVisibilityChecked = false;
    areTargetDirectionsFound = false;
    isAssociationComputed = false;

    data.clear();

    originalMesh.clear();
    smoothedMesh.clear();
}




/* ----- COMPUTING METHODS ------ */


/**
 * @brief Compute entire algorithm
 */
void FourAxisFabricationManager::computeEntireAlgorithm() {
    //Get UI data
    unsigned int nOrientation = (unsigned int) ui->nOrientationSpinBox->value();
    bool deterministic = ui->deterministicCheckBox->isChecked();
    unsigned int nDirections = (unsigned int) ui->nDirectionsSpinBox->value();
    bool fixExtremeAssociation = ui->fixExtremeAssociationCheckBox->isChecked();
    FourAxisFabrication::CheckMode checkMode = (ui->rayShootingRadio->isChecked() ?
            FourAxisFabrication::RAYSHOOTING :
            FourAxisFabrication::PROJECTION);
    bool setCoverageFlag = ui->setCoverageCheckBox->isChecked();

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
                data,
                checkMode);


    t.stopAndPrint();

    isMeshOriented = true;
    areExtremesCut = true;
    isVisibilityChecked = true;
    areTargetDirectionsFound = true;
    isAssociationComputed = true;
}

/**
 * @brief Compute optimal orientation
 */
void FourAxisFabricationManager::optimalOrientation() {
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
}

/**
 * @brief Cut extremes
 */
void FourAxisFabricationManager::cutExtremes() {
    cg3::Timer t("Cut extremes");

    //Get extremes on x-axis to be cut
    FourAxisFabrication::getExtremesOnXAxis(smoothedMesh, data);

    t.stopAndPrint();

    areExtremesCut = true;
}

/**
 * @brief Check visibility from various directions
 */
void FourAxisFabricationManager::checkVisibility() {
    //Get UI data
    unsigned int nDirections = (unsigned int) ui->nDirectionsSpinBox->value();
    bool fixExtremeAssociation = ui->fixExtremeAssociationCheckBox->isChecked();
    FourAxisFabrication::CheckMode checkMode = (ui->rayShootingRadio->isChecked() ?
            FourAxisFabrication::RAYSHOOTING :
            FourAxisFabrication::PROJECTION);


    cg3::Timer t("Visibility check");

    //Initialize data before visibility check
    FourAxisFabrication::initializeDataForVisibilityCheck(
                smoothedMesh,
                nDirections,
                fixExtremeAssociation,
                data);

    //Visibility check
    FourAxisFabrication::checkVisibility(
                smoothedMesh,
                nDirections,
                data,
                checkMode);

    //Detect non-visible faces
    FourAxisFabrication::detectNonVisibleFaces(
                data);

    t.stopAndPrint();

    isVisibilityChecked = true;
}

/**
 * @brief Get the target directions
 */
void FourAxisFabricationManager::getTargetDirections() {
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

/**
 * @brief Get optimized association
 */
void FourAxisFabricationManager::getAssociation() {
    //Get UI data

    cg3::Timer t("Get association");

    //Get association
    FourAxisFabrication::getOptimizedAssociation(
                smoothedMesh,
                data);

    t.stopAndPrint();

    isAssociationComputed = true;
}





/* ----- VISUALIZATION METHODS ------ */

/**
 * @brief Reset camera pointing to the z-axis direction
 */
void FourAxisFabricationManager::resetCameraDirection() {
    mainWindow.setCameraDirection(cg3::Vec3(-1,0,0));
    mainWindow.setCameraDirection(cg3::Vec3(0,0,-1));

    mainWindow.fitScene();
    mainWindow.updateGlCanvas();
}

/**
 * @brief Set camera direction
 */
void FourAxisFabricationManager::setCameraDirection(cg3::Vec3 dir) {
    mainWindow.setCameraDirection(dir);

    mainWindow.fitScene();
    mainWindow.updateGlCanvas();
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
    mainWindow.updateGlCanvas();

    updateUI();
}

/**
 * @brief Colorize mesh to the default color
 */
void FourAxisFabricationManager::visualizeMesh() {
    //Default color
    smoothedMesh.setFaceColor(cg3::Color(128,128,128));

    ui->descriptionLabel->setText(""); //Empty description text
}


/**
 * @brief Colorize the min and max extremes
 */
void FourAxisFabricationManager::visualizeExtremes() {
    //Default color
    smoothedMesh.setFaceColor(cg3::Color(128,128,128));


    unsigned int sliderValue = (unsigned int) ui->visualizationSlider->value();

    //Color the min and max extremes
    if (sliderValue == 0 || sliderValue == 1) {
        for (unsigned int i : data.minExtremes){
            smoothedMesh.setFaceColor(cg3::Color(0,0,255), i);
        }
    }
    if (sliderValue == 0 || sliderValue == 2) {
        for (unsigned int i : data.maxExtremes){
            smoothedMesh.setFaceColor(cg3::Color(255,0,0), i);
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
    smoothedMesh.setFaceColor(cg3::Color(128,128,128));

    std::stringstream ss;

    //Color the not visible
    if (sliderValue == 0) {
        for (unsigned int faceId : data.nonVisibleFaces) {
            smoothedMesh.setFaceColor(cg3::Color(0,0,0), faceId);
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
                smoothedMesh.setFaceColor(color, j);
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
    smoothedMesh.setFaceColor(cg3::Color(128,128,128));

    std::stringstream ss;

    //Color the not visible
    if (sliderValue == 0) {
        for (unsigned int faceId : data.nonVisibleFaces) {
            smoothedMesh.setFaceColor(cg3::Color(0,0,0), faceId);
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
                smoothedMesh.setFaceColor(color, j);
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
 * @brief Colorize optmized association
 */
void FourAxisFabricationManager::visualizeAssociation() {
    unsigned int sliderValue = (unsigned int) ui->visualizationSlider->value();

    //Set default color
    smoothedMesh.setFaceColor(cg3::Color(128,128,128));

    std::stringstream ss;


    //Subdivisions for colors
    int subd = 255 / (data.targetDirections.size()-1);


    //For each face check if it is visible by that plane
    for (unsigned int faceId = 0; faceId < smoothedMesh.getNumberFaces(); faceId++) {
        //Get direction index associated to the current face
        int associatedDirectionIndex = data.association[faceId];

        //If it has an associated fabrication direction
        if (associatedDirectionIndex >= 0) {
            //Find position in target directions to set the color
            std::vector<unsigned int>::iterator it =
                    std::find(data.targetDirections.begin(), data.targetDirections.end(), associatedDirectionIndex);

            int positionInTargetDirections = std::distance(data.targetDirections.begin(), it);

            cg3::Color color;
            color.setHsv(subd * positionInTargetDirections, 255, 255);

            if (sliderValue == 0 ||
                    data.targetDirections[sliderValue-1] == associatedDirectionIndex)
            {
                //Set the color
                smoothedMesh.setFaceColor(color, faceId);
            }
        }
        else {
            //Black color for non-visible faces
            smoothedMesh.setFaceColor(cg3::Color(0,0,0), faceId);
        }
    }

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
                    //Add meshes to the canvas, hiding the original one
                    std::string meshName = meshFile.substr(meshFile.find_last_of("/") + 1);
                    mainWindow.pushObj(&originalMesh, meshName);
                    mainWindow.pushObj(&smoothedMesh, "Smoothed mesh");
                    mainWindow.setObjVisibility(&originalMesh, false);

                    loadedMeshFile = meshFile;
                    loadedSmoothedMeshFile = smoothedFile;

                    //Visualize mesh
                    ui->meshRadio->setChecked(true);
                    initializeVisualizationSlider();

                    //Update canvas and fit the scene
                    mainWindow.updateGlCanvas();
                    mainWindow.fitScene();

                    std::cout << std::endl << "Mesh file \"" << meshName << "\" loaded. Number of faces: " << originalMesh.getNumberFaces() << std::endl;
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
        //Clear four axis fabrication data
        clearData();

        //Clear file names
        loadedMeshFile = "";
        loadedSmoothedMeshFile = "";

        //Delete objects and update canvas
        mainWindow.deleteObj(&originalMesh);
        mainWindow.deleteObj(&smoothedMesh);


        //Update canvas and fit the scene
        mainWindow.updateGlCanvas();
        mainWindow.fitScene();        

        //Visualize mesh
        ui->meshRadio->setChecked(true);
        initializeVisualizationSlider();

        updateUI();
    }

}

void FourAxisFabricationManager::on_reloadMeshButton_clicked()
{
    if (isMeshLoaded) {
        //Delete objects and update canvas
        mainWindow.deleteObj(&originalMesh);
        mainWindow.deleteObj(&smoothedMesh);

        //Clear four axis fabrication data
        clearData();


        //Try to reload the meshes
        isMeshLoaded =
                originalMesh.readFromObj(loadedMeshFile) &
                smoothedMesh.readFromObj(loadedSmoothedMeshFile);

        //If the meshes have been successfully loaded
        if (isMeshLoaded){
            //Add meshes to the canvas, hiding the original one
            std::string meshName = loadedMeshFile.substr(loadedMeshFile.find_last_of("/") + 1);
            mainWindow.pushObj(&originalMesh, meshName);
            mainWindow.pushObj(&smoothedMesh, "Smoothed mesh");
            mainWindow.setObjVisibility(&originalMesh, false);
        }
        else {
            clearData();
        }

        //Update canvas and fit the scene
        mainWindow.updateGlCanvas();
        mainWindow.fitScene();

        //Visualize mesh
        ui->meshRadio->setChecked(true);
        initializeVisualizationSlider();

        updateUI();
        resetCameraDirection();
    }
}


/* ----- UI SLOTS FOUR AXIS FABRICATION ------ */


void FourAxisFabricationManager::on_computeEntireAlgorithmButton_clicked() {
    if (isMeshLoaded){
        std::cout << std::endl;

        //Compute the entire algorithm
        computeEntireAlgorithm();

        //Visualize association
        ui->associationRadio->setChecked(true);
        initializeVisualizationSlider();

        //Update canvas and fit the scene
        mainWindow.updateGlCanvas();
        mainWindow.fitScene();

        updateUI();
    }
}


void FourAxisFabricationManager::on_optimalOrientationButton_clicked() {
    if (isMeshLoaded){
        std::cout << std::endl;

        //Get optimal mesh orientation
        optimalOrientation();

        //Visualize mesh
        ui->meshRadio->setChecked(true);
        initializeVisualizationSlider();

        //Update canvas and fit the scene
        mainWindow.updateGlCanvas();
        mainWindow.fitScene();

        updateUI();
    }
}

void FourAxisFabricationManager::on_cutExtremesButton_clicked()
{
    if (isMeshLoaded && isMeshOriented){
        //Get extremes on x-axis to be cut
        cutExtremes();

        //Visualize extremes
        ui->extremesRadio->setChecked(true);
        initializeVisualizationSlider();

        //Update canvas and fit the scene
        mainWindow.updateGlCanvas();

        updateUI();
    }
}

void FourAxisFabricationManager::on_checkVisibilityButton_clicked()
{
    if (isMeshLoaded && areExtremesCut){
        //Check visibility by the chosen directions
        checkVisibility();

        //Visualize visibility
        ui->visibilityRadio->setChecked(true);
        initializeVisualizationSlider();

        //Update canvas and fit the scene
        mainWindow.updateGlCanvas();

        updateUI();
    }
}


void FourAxisFabricationManager::on_targetDirectionsButton_clicked()
{
    if (isMeshLoaded && areExtremesCut && isVisibilityChecked){
        //Get target milling directions
        getTargetDirections();

        //Visualize target directions
        ui->targetDirectionsRadio->setChecked(true);
        initializeVisualizationSlider();

        //Update canvas and fit the scene
        mainWindow.updateGlCanvas();

        updateUI();
    }
}

void FourAxisFabricationManager::on_getAssociationButton_clicked()
{
    if (isMeshLoaded && areExtremesCut && isVisibilityChecked && areTargetDirectionsFound){
        //Check visibility by the chosen directions
        getAssociation();

        //Visualize association
        ui->associationRadio->setChecked(true);
        initializeVisualizationSlider();

        //Update canvas and fit the scene
        mainWindow.updateGlCanvas();

        updateUI();
    }
}





/* ----- UI SLOTS TRANSFORMATIONS ------ */


void FourAxisFabricationManager::on_centerOnOriginButton_clicked() {
    if (isMeshLoaded){
        //Translation of the mesh
        originalMesh.translate(-smoothedMesh.getBoundingBox().center());
        smoothedMesh.translate(-smoothedMesh.getBoundingBox().center());

        //Update canvas and fit the scene
        mainWindow.updateGlCanvas();
        mainWindow.fitScene();
    }
}

void FourAxisFabricationManager::on_plusXButton_clicked() {
    if (isMeshLoaded){
        //Translation of the mesh
        originalMesh.translate(cg3::Pointd(ui->stepSpinBox->value(), 0, 0));
        smoothedMesh.translate(cg3::Pointd(ui->stepSpinBox->value(), 0, 0));

        //Update canvas and fit the scene
        mainWindow.updateGlCanvas();
        mainWindow.fitScene();
    }
}

void FourAxisFabricationManager::on_minusXButton_clicked() {
    if (isMeshLoaded){
        //Translation of the mesh
        originalMesh.translate(cg3::Pointd(-ui->stepSpinBox->value(), 0, 0));
        smoothedMesh.translate(cg3::Pointd(-ui->stepSpinBox->value(), 0, 0));

        //Update canvas and fit the scene
        mainWindow.updateGlCanvas();
        mainWindow.fitScene();
    }
}

void FourAxisFabricationManager::on_plusYButton_clicked() {
    if (isMeshLoaded){
        //Translation of the mesh
        originalMesh.translate(cg3::Pointd(0, ui->stepSpinBox->value(), 0));
        smoothedMesh.translate(cg3::Pointd(0, ui->stepSpinBox->value(), 0));

        //Update canvas and fit the scene
        mainWindow.updateGlCanvas();
        mainWindow.fitScene();
    }
}

void FourAxisFabricationManager::on_minusYButton_clicked() {
    if (isMeshLoaded){
        //Translation of the mesh
        originalMesh.translate(cg3::Pointd(0, -ui->stepSpinBox->value(), 0));
        smoothedMesh.translate(cg3::Pointd(0, -ui->stepSpinBox->value(), 0));

        //Update canvas and fit the scene
        mainWindow.updateGlCanvas();
        mainWindow.fitScene();
    }
}

void FourAxisFabricationManager::on_plusZButton_clicked() {
    if (isMeshLoaded){
        //Translation of the mesh
        originalMesh.translate(cg3::Pointd(0, 0, ui->stepSpinBox->value()));
        smoothedMesh.translate(cg3::Pointd(0, 0, ui->stepSpinBox->value()));

        //Update canvas and fit the scene
        mainWindow.updateGlCanvas();
        mainWindow.fitScene();
    }
}

void FourAxisFabricationManager::on_minusZButton_clicked() {
    if (isMeshLoaded){
        //Translation of the mesh
        originalMesh.translate(cg3::Pointd(0, 0, -ui->stepSpinBox->value()));
        smoothedMesh.translate(cg3::Pointd(0, 0, -ui->stepSpinBox->value()));

        //Update canvas and fit the scene
        mainWindow.updateGlCanvas();
        mainWindow.fitScene();
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

        originalMesh.updateFaceNormals();
        smoothedMesh.updateFaceNormals();

        //Update canvas and fit the scene
        mainWindow.updateGlCanvas();
        mainWindow.fitScene();
    }
}




void FourAxisFabricationManager::on_scaleButton_clicked() {
    if (isMeshLoaded){
        //Scale the mesh
        cg3::Vec3 scaleFactor(ui->scaleXSpinBox->value(), ui->scaleYSpinBox->value(), ui->scaleZSpinBox->value());

        originalMesh.scale(scaleFactor);
        smoothedMesh.scale(scaleFactor);

        //Update canvas and fit the scene
        mainWindow.updateGlCanvas();
        mainWindow.fitScene();
    }
}

void FourAxisFabricationManager::on_inverseScaleButton_clicked() {
    if (isMeshLoaded){
        //Scale the mesh
        cg3::Vec3 scaleFactor(1.0/ui->scaleXSpinBox->value(), 1.0/ui->scaleYSpinBox->value(), 1.0/ui->scaleZSpinBox->value());

        originalMesh.scale(scaleFactor);
        smoothedMesh.scale(scaleFactor);

        //Update canvas and fit the scene
        mainWindow.updateGlCanvas();
        mainWindow.fitScene();
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

