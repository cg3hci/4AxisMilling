#ifndef PAINTINGWINDOWN_H
#define PAINTINGWINDOWN_H

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

class PaintingWindown
{

public:

    //PaintingWindown(std::vector<std::vector<size_t>>& value, std::string meshName);
    PaintingWindown(std::vector<std::vector<size_t> > &value, cg3::DrawableEigenMesh &mesh);

    void setInstance(std::string meshName);

    void showWindown();

    std::vector<std::vector<size_t>>& partitions;

private:

    void setWindown(bool show);

    void loadMesh(std::string meshName);

    void loadEigenMesh();

    void connectButtons();

    void pushObjectCanvas();

    std::string generateModelPath(bool openDocumentFolder,
                                  std::string meshName);

    uint closest_vertex_paint(const cinolib::vec3d & p);


    void createChartFromSeed(cinolib::ScalarField &f,
                             float brush_size, std::set<uint> &currentChart);

    void mergeCharts(std::set<uint>& currentChart);


    QWidget     window;
    QVBoxLayout layout;
    cinolib::GLcanvas canvas;
    QSlider sl_size;
    QPushButton but_reset;
    QPushButton but_next;
    cg3::viewer::LoaderSaver    loaderSaverObj;
    cinolib::DrawableTrimesh<>  meshToPaint;
    cg3::DrawableEigenMesh& drawablePaintedMesh;
    cinolib::GeodesicsCache     prefactored_matrices;
    std::vector<std::set<uint>> chartFaces;

};

#endif // PAINTINGWINDOWN_H
