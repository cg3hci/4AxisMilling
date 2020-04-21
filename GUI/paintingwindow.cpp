#include "paintingwindow.h"

PaintingWindow::PaintingWindow(QWidget* parent) :
    QWidget(parent)
{
    window = new QWidget();
    layout = new QVBoxLayout();
    canvas = new cinolib::GLcanvas();
    sl_size = new QSlider();
    but_confirm = new QPushButton();

    buildUi();
    connectButtons();

    canvas->push_obj(&meshToPaint);
    meshToPaint.show_wireframe(true);
    meshToPaint.show_poly_color();

    clearData();
}

void PaintingWindow::showWindow()
{
    window->showMaximized();
}

void PaintingWindow::loadData(const cg3::EigenMesh* mesh, std::vector<double>* faceSaliency, double minSaliency, double maxSaliency)
{
    clearData();

    this->mesh = mesh;
    this->faceSaliency = faceSaliency;
    this->minSaliency = minSaliency;
    this->maxSaliency = maxSaliency;

    for(uint vertexId = 0; vertexId < mesh->numberVertices(); vertexId++) {
        meshToPaint.vert_add(cinolib::vec3d(mesh->vertex(vertexId).x(), mesh->vertex(vertexId).y(), mesh->vertex(vertexId).z()));
    }

    for(uint faceId = 0; faceId < mesh->numberFaces(); faceId++) {
        meshToPaint.poly_add(mesh->face(faceId).x(), mesh->face(faceId).y(), mesh->face(faceId).z());
    }

    initGeodesics();

    colorizeFacesBySaliency();
}

void PaintingWindow::clearData()
{
    mesh = nullptr;
    faceSaliency = nullptr;

    prefactored_matrices = cinolib::GeodesicsCache();

    meshToPaint.clear();
    meshToPaint.updateGL();

    canvas->updateGL();
}

void PaintingWindow::buildUi()
{
    canvas->setParent(window);

    but_confirm->setText("Confirm");
    but_confirm->setParent(window);

    sl_size->setOrientation(Qt::Horizontal);
    sl_size->setParent(window);
    sl_size->setMaximum(100);
    sl_size->setMinimum(0);
    sl_size->setValue(8);

    layout->addWidget(sl_size);
    layout->addWidget(but_confirm);
    layout->addWidget(canvas);
    window->setLayout(layout);

    window->resize(1024,1024);
    window->setWindowState(Qt::WindowMaximized); //Maximizes the window
    window->setWindowFlags(Qt::WindowTitleHint | Qt::WindowMinimizeButtonHint | Qt::WindowMaximizeButtonHint); //Hide X button
}

void PaintingWindow::connectButtons(){
    QPushButton::connect(but_confirm, &QPushButton::clicked, [&]()
    {
        canvas->updateGL();
        window->close();

        Q_EMIT meshPainted();
    });

    // CMD+1 to show mesh controls.
    cinolib::SurfaceMeshControlPanel< cinolib::DrawableTrimesh<>> panel(&meshToPaint, canvas);
    QApplication::connect(new QShortcut(QKeySequence(Qt::CTRL+Qt::Key_1), canvas), &QShortcut::activated, [&](){panel.show();});
}

void PaintingWindow::initGeodesics()
{
    compute_geodesics_amortized(meshToPaint, prefactored_matrices, {0});
    canvas->callback_mouse_press = [&](cinolib::GLcanvas *c, QMouseEvent *e)
    {
        cinolib::vec3d p;

        if (e->modifiers() == Qt::ControlModifier) {
            if(c->unproject(cinolib::vec2i(e->x(),e->y()), p)) {
                createChart(p, true);
                meshToPaint.updateGL();
                c->updateGL();
            }
        }

        if (e->modifiers() == Qt::ShiftModifier) {
            if(c->unproject(cinolib::vec2i(e->x(),e->y()), p)) {
                createChart(p, false);
                meshToPaint.updateGL();
                c->updateGL();
            }
        }
        return;
    };
}

uint PaintingWindow::closest_vertex_paint(const cinolib::vec3d & p){
    std::vector<std::pair<double,uint>> closest;
    for(uint vid=0; vid<meshToPaint.num_verts(); ++vid)
        closest.push_back(std::make_pair(meshToPaint.vert(vid).dist(p),vid));
    std::sort(closest.begin(), closest.end());
    return closest.front().second;
}

void PaintingWindow::createChart(
        const cinolib::vec3d& p,
        const bool selectMode)
{
    float brush_size = static_cast<float>(sl_size->value())/100.f;
    uint  vertexId = closest_vertex_paint(p);

    cinolib::ScalarField f = compute_geodesics_amortized(meshToPaint, prefactored_matrices, {vertexId});

    for(uint fId = 0; fId < meshToPaint.num_polys(); ++fId) {
        //TODO Cercare il vertico più vicino del triangolo
        std::vector<float> closest;
        closest.push_back(f[meshToPaint.poly_vert_id(fId,0)]);
        closest.push_back(f[meshToPaint.poly_vert_id(fId,1)]);
        closest.push_back(f[meshToPaint.poly_vert_id(fId,2)]);
        std::sort(closest.begin(), closest.end());

        float dist = 1.f - closest.front();
        //float dist = 1.f - f[meshToPaint.poly_vert_id(faceId,0)];

        if (dist <= brush_size)
        {
            //Gradiente
            //float val = meshToPaint.poly_data(faceId).color.g;
            //val -= (brush_size-dist)/brush_size;
            //if(val<0) val = 0.f;
            if (selectMode) {
                faceSaliency->at(fId) = MAXSALIENCY;
            }
            else {
                faceSaliency->at(fId) = 0.0;
            }
        }
    }

    colorizeFacesBySaliency();
}

void PaintingWindow::colorizeFacesBySaliency() {
    for(uint fId = 0; fId < meshToPaint.num_polys(); ++fId) {
        double normalizedValue = (faceSaliency->at(fId) - minSaliency)/(maxSaliency - minSaliency);

        cinolib::Color color = getColorBySaliency(normalizedValue);
        meshToPaint.poly_data(fId).color = color;
    }

    meshToPaint.updateGL();
    canvas->updateGL();
}


cinolib::Color PaintingWindow::getColorBySaliency(const double value)
{
    cinolib::Color color(0,0,0);

    if (value <= 0) {
        color.r = 1.0;
    }
    else if (value >= 1) {
        color.b = 1.0;
    }
    else if (value <= 0.5f) {
        double normalizedValue = value * 2.0;
        color.r = 1.0 - normalizedValue;
        color.g = normalizedValue;
    }
    else {
        double normalizedValue = (value - 0.5) * 2.0;
        color.g = 1.0 - normalizedValue;
        color.b = normalizedValue;
    }

    return color;
}
