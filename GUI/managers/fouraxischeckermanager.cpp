#include "fouraxischeckermanager.h"
#include "ui_fouraxischeckermanager.h"

#include <cg3/geometry/transformations.h>
#include <cg3/utilities/string.h>

#include <QtGui>
#include <string>
#include <QDebug>

#include "../fouraxis/fouraxis.h"
#include <cg3/meshes/eigenmesh/algorithms/eigenmesh_algorithms.h>

#define halfC (M_PI / 180)

FourAxisMillingManager::FourAxisMillingManager(QWidget *parent) :
    QFrame(parent),
    ui(new Ui::FourAxisMillingManager),
    mainWindow((MainWindow&)*parent)

{
    ui->setupUi(this);
    connect(&mainWindow, SIGNAL(objectPicked(unsigned int)),this, SLOT(on_triangleClicked(unsigned int)));
    objls.addSupportedExtension("obj");
}

FourAxisMillingManager::~FourAxisMillingManager(){
    delete ui;
}

void FourAxisMillingManager::on_checkPushButton_clicked() {
    if (loaded) {
        smoothedMesh.setFaceColor(cg3::Color(128,128,128));
        cg3::Array2D<int> visibility;
        //std::vector<int> survivedPlanes;
        int nPlaneUser = 10;
        //int nPlaneUser = ui->nPlane->text().toInt(); //TODO
        FourAxisFabrication::checkVisibilityAllPlanes(smoothedMesh, visibility, nPlaneUser);
        for (unsigned int j = 0; j < smoothedMesh.getNumberFaces(); j++)
            if (visibility(visibility.getSizeX()-1, j) == 1)
                smoothedMesh.setFaceColor(cg3::Color(255,0,0), j);

        //FourAxisFabrication::minimizeNumberPlanes(survivedPlanes, visibility);
        #ifdef MULTI_LABEL_OPTIMIZATION_INCLUDED
        std::vector<int> ass = FourAxisFabrication::getAssociation(survivedPlanes, visibility, smoothedMesh);
        //to know the actual orientation: survivedPlanes[ass[f]]
        int subd = 240 / survivedPlanes.size();
        for (unsigned int i = 0; i < ass.size(); i++){
            Color c;
            if (ass[i] == nPlaneUser*2)
                c = Color(0,0,0);
            else
                c.setHsv(subd*ass[i], 255, 255);
            smoothedMesh.setFaceColor(c, i);
        }
        #endif
        mainWindow.updateGlCanvas();
    }
}

void FourAxisMillingManager::on_saveMeshAxis_clicked() {
    if (loaded){
        originalMesh.saveOnObj("rotatedMesh.obj");
        cg3::EigenMeshAlgorithms::makeCylinder(cg3::Pointd(originalMesh.getBoundingBox().minX()-5,0,0), cg3::Pointd(originalMesh.getBoundingBox().maxX()+5,0,0), 0.5).saveOnObj("rotationCylinder.obj");
    }
}

void FourAxisMillingManager::on_loadMeshPushButton_clicked() {
    std::string filename = objls.loadDialog("Load Mesh");
    if (filename != "") {
        loaded = originalMesh.readFromObj(filename);
        if (loaded){
            std::string rawname, ext;
            cg3::separateExtensionFromFilename(filename, rawname, ext);
            loaded = smoothedMesh.readFromObj(rawname + "_smooth" + ext);
            if (!loaded){
                std::string filenameS = objls.loadDialog("Load Smoothed Mesh");
                if (filenameS != ""){
                    loaded = smoothedMesh.readFromObj(filenameS);
                }
            }
            if (loaded) {
                mainWindow.pushObj(&originalMesh, filename.substr(filename.find_last_of("/") + 1));
                mainWindow.pushObj(&smoothedMesh, "Smoothed Mesh");
                mainWindow.setObjVisibility(&originalMesh, false);
                mainWindow.updateGlCanvas();
            }
        }
    }
}

void FourAxisMillingManager::on_plusXButton_clicked() {
    if (loaded){
        originalMesh.translate(cg3::Pointd(ui->stepSpinBox->value(), 0, 0));
        smoothedMesh.translate(cg3::Pointd(ui->stepSpinBox->value(), 0, 0));
        mainWindow.updateGlCanvas();
    }
}

void FourAxisMillingManager::on_minusXButton_clicked() {
    if (loaded){
        originalMesh.translate(cg3::Pointd(-ui->stepSpinBox->value(), 0, 0));
        smoothedMesh.translate(cg3::Pointd(-ui->stepSpinBox->value(), 0, 0));
        mainWindow.updateGlCanvas();
    }
}

void FourAxisMillingManager::on_plusYButton_clicked() {
    if (loaded){
        originalMesh.translate(cg3::Pointd(0, ui->stepSpinBox->value(), 0));
        smoothedMesh.translate(cg3::Pointd(0, ui->stepSpinBox->value(), 0));
        mainWindow.updateGlCanvas();
    }
}

void FourAxisMillingManager::on_minusYButton_clicked() {
    if (loaded){
        originalMesh.translate(cg3::Pointd(0, -ui->stepSpinBox->value(), 0));
        smoothedMesh.translate(cg3::Pointd(0, -ui->stepSpinBox->value(), 0));
        mainWindow.updateGlCanvas();
    }
}

void FourAxisMillingManager::on_plusZButton_clicked() {
    if (loaded){
        originalMesh.translate(cg3::Pointd(0, 0, ui->stepSpinBox->value()));
        smoothedMesh.translate(cg3::Pointd(0, 0, ui->stepSpinBox->value()));
        mainWindow.updateGlCanvas();
    }
}

void FourAxisMillingManager::on_minusZButton_clicked() {
    if (loaded){
        originalMesh.translate(cg3::Pointd(0, 0, -ui->stepSpinBox->value()));
        smoothedMesh.translate(cg3::Pointd(0, 0, -ui->stepSpinBox->value()));
        mainWindow.updateGlCanvas();
    }
}

void FourAxisMillingManager::on_rotateButton_clicked() {
    if (loaded){
        cg3::Vec3 axis(ui->axisXSpinBox->value(), ui->axisYSpinBox->value(), ui->axisZSpinBox->value());
        double angle = ui->angleSpinBox->value()*M_PI/180;
        Eigen::Matrix3d m;
        cg3::getRotationMatrix(axis, angle, m);
        originalMesh.rotate(m);
        smoothedMesh.rotate(m);
        mainWindow.updateGlCanvas();
    }
}

void FourAxisMillingManager::on_scalePushButton_clicked() {
    if (loaded){
        cg3::Vec3 scaleFactor(ui->scaleXSpinBox->value(), ui->scaleYSpinBox->value(), ui->scaleZSpinBox->value());
        originalMesh.scale(scaleFactor);
        smoothedMesh.scale(scaleFactor);
        mainWindow.updateGlCanvas();
    }
}

void FourAxisMillingManager::on_inverseScalePushButton_clicked() {
    if (loaded){
        cg3::Vec3 scaleFactor(1.0/ui->scaleXSpinBox->value(), 1.0/ui->scaleYSpinBox->value(), 1.0/ui->scaleZSpinBox->value());
        originalMesh.scale(scaleFactor);
        smoothedMesh.scale(scaleFactor);
        mainWindow.updateGlCanvas();
    }
}

void FourAxisMillingManager::on_meshToOriginPushButton_clicked() {
    if (loaded){
        originalMesh.translate(-smoothedMesh.getBoundingBox().center());
        smoothedMesh.translate(-smoothedMesh.getBoundingBox().center());
        mainWindow.updateGlCanvas();
    }
}

void FourAxisMillingManager::on_clearMeshPushButton_clicked() {
    if (loaded) {
        mainWindow.deleteObj(&originalMesh);
        mainWindow.deleteObj(&smoothedMesh);
        loaded = false;
        mainWindow.updateGlCanvas();
    }
}

void FourAxisMillingManager::on_automaticOrientationPushButton_clicked() {
    if (loaded){
        FourAxisFabrication::findOptimalRotation(smoothedMesh, originalMesh);
        mainWindow.updateGlCanvas();
    }
}
