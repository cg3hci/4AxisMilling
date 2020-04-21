#ifndef PAINTINGWINDOW_H
#define PAINTINGWINDOW_H

#include <QWidget>
#include <QSlider>
#include <QVBoxLayout>
#include <QPushButton>
#include <QColor>
#include <QApplication>
#include <QMainWindow>
#include <cinolib/gui/qt/qt_gui_tools.h>
#include <cinolib/meshes/meshes.h>
#include <cinolib/geodesics.h>

#include <cg3/viewer/utilities/loadersaver.h>
#include <cg3/meshes/eigenmesh/eigenmesh.h>

#define MAXSALIENCY 100.0

class PaintingWindow : public QMainWindow
{
    Q_OBJECT

public:

    PaintingWindow(QWidget* parent = nullptr);

    void showWindow();

    void loadData(const cg3::EigenMesh* mesh, std::vector<double>* faceSaliency, double minSaliency, double maxSaliency);
    void clearData();


Q_SIGNALS:

    void meshPainted();


private:

    void setWindow(bool show);
    void initGeodesics();
    void connectButtons();

    uint closest_vertex_paint(const cinolib::vec3d& p);

    void createChart(
            const cinolib::vec3d& p,
            const bool selectMode);

    void colorizeFacesBySaliency();
    cinolib::Color getColorBySaliency(const double value);

    const cg3::EigenMesh* mesh;
    std::vector<double>* faceSaliency;
    double minSaliency;
    double maxSaliency;

    QWidget*     window;
    QVBoxLayout* layout;
    cinolib::GLcanvas* canvas;
    QSlider* sl_size;
    QPushButton* but_confirm;
    cinolib::DrawableTrimesh<>  meshToPaint;
    cinolib::GeodesicsCache     prefactored_matrices;

public:

    void closeEvent(QCloseEvent *event);

};

#endif // PAINTINGWINDOW_H
