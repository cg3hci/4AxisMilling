#ifndef PAINTINGWINDOW_H
#define PAINTINGWINDOW_H

#include <QWidget>
#include <QSlider>
#include <QVBoxLayout>
#include <QPushButton>
#include <QColor>
#include <QApplication>
#include <cinolib/gui/qt/qt_gui_tools.h>
#include <cinolib/meshes/meshes.h>
#include <cinolib/geodesics.h>

#include <cg3/viewer/utilities/loadersaver.h>
#include <cg3/viewer/drawable_objects/drawable_eigenmesh.h>

class PaintingWindow : public QObject
{
    Q_OBJECT

public:

    PaintingWindow(std::vector<bool>& paintedFaces, const cg3::DrawableEigenMesh& mesh);

    void setInstance(std::string meshName);

    void showWindow();

    std::vector<std::vector<size_t>> partitions;

    std::vector<bool>& paintedFaces;


Q_SIGNALS:

    void meshPainted();


private:

    void setWindow(bool show);

    void loadMesh(std::string meshName);

    void loadEigenMesh();

    void connectButtons();

    void pushObjectCanvas();

    std::string generateModelPath(bool openDocumentFolder,
                                  std::string meshName);

    uint closest_vertex_paint(const cinolib::vec3d & p);

    void createChart(cinolib::vec3d p, bool paint);

    void mergeCharts(std::set<uint>& currentChart);

    QWidget*     window;
    QVBoxLayout* layout;
    cinolib::GLcanvas* canvas;
    QSlider* sl_size;
    QPushButton* but_reset;
    QPushButton* but_close;
    cg3::viewer::LoaderSaver    loaderSaverObj;
    cinolib::DrawableTrimesh<>  meshToPaint;
    const cg3::DrawableEigenMesh& drawablePaintedMesh;
    cinolib::GeodesicsCache     prefactored_matrices;
    std::vector<std::set<uint>> chartFaces;

};

#endif // PAINTINGWINDOW_H
