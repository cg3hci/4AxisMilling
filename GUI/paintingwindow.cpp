#include "paintingwindow.h"

PaintingWindow::PaintingWindow(std::vector<bool>& paintedFaces, const cg3::DrawableEigenMesh& mesh) :
    paintedFaces(paintedFaces),
    drawablePaintedMesh(mesh)
{
    window = new QWidget();
    layout = new QVBoxLayout();
    canvas = new cinolib::GLcanvas();
    sl_size = new QSlider();
    but_reset = new QPushButton();
    but_close = new QPushButton();

    setWindow(false);
}

void PaintingWindow::setInstance(std::string meshName){
    if(meshName == "") loadEigenMesh();
    else loadMesh(meshName);
}

void PaintingWindow::showWindow(){
    window->show();
}

void PaintingWindow::setWindow(bool show){

    canvas->setParent(window);
    but_reset->setText("Reset");
    but_reset->setParent(window);
    but_close->setText("Confirm and close");
    but_close->setParent(window);
    sl_size->setOrientation(Qt::Horizontal);
    sl_size->setParent(window);
    sl_size->setMaximum(100);
    sl_size->setMinimum(0);
    sl_size->setValue(10);
    layout->addWidget(sl_size);
    layout->addWidget(but_reset);
    layout->addWidget(but_close);
    layout->addWidget(canvas);
    window->setLayout(layout);
    if(show)
        window->show();
    window->resize(1024,1024);
}

void PaintingWindow::loadMesh(std::string meshName){
    meshToPaint = cinolib::DrawableTrimesh<>(meshName.c_str());
    pushObjectCanvas();

    connectButtons();

}

void PaintingWindow::loadEigenMesh(){

    meshToPaint.clear();
    //paintedFaces.clear();
    if(paintedFaces.empty())
        paintedFaces.resize(drawablePaintedMesh.numberFaces(), false);

    for(uint vertexId = 0; vertexId < drawablePaintedMesh.numberVertices(); vertexId++){
        meshToPaint.vert_add(cinolib::vec3d(drawablePaintedMesh.vertex(vertexId).x(), drawablePaintedMesh.vertex(vertexId).y(), drawablePaintedMesh.vertex(vertexId).z()));
    }

    for(uint faceId = 0; faceId < drawablePaintedMesh.numberFaces(); faceId++){
        meshToPaint.poly_add(drawablePaintedMesh.face(faceId).x(), drawablePaintedMesh.face(faceId).y(), drawablePaintedMesh.face(faceId).z());
        meshToPaint.poly_data(faceId).color = cinolib::Color(drawablePaintedMesh.faceColor(faceId).red()/128, drawablePaintedMesh.faceColor(faceId).green()/128, drawablePaintedMesh.faceColor(faceId).blue()/128);
    }

    pushObjectCanvas();

    connectButtons();

}

void PaintingWindow::connectButtons(){
    QPushButton::connect(but_reset, &QPushButton::clicked, [&]()
    {
        meshToPaint.poly_set_color( cinolib::Color(0.5,0.5,0.5));

        canvas->updateGL();
    });
    QPushButton::connect(but_close, &QPushButton::clicked, [&]()
    {
        //Create partition
        /*partitions.clear();
        std::cout << chartFaces.size() << std::endl;
        for(uint chartId = 0; chartId < chartFaces.size(); chartId++){
            std::vector<size_t> currentChart;
            for(std::set<uint>::iterator it = chartFaces[chartId].begin(); it != chartFaces[chartId].end(); ++it){
                currentChart.push_back(*it);
            }
            partitions.push_back(currentChart);
        }*/

        canvas->updateGL();
        window->close();

        Q_EMIT meshPainted();
    });


    // CMD+1 to show mesh controls.
    cinolib::SurfaceMeshControlPanel< cinolib::DrawableTrimesh<>> panel(&meshToPaint, canvas);
    QApplication::connect(new QShortcut(QKeySequence(Qt::CTRL+Qt::Key_1), canvas), &QShortcut::activated, [&](){panel.show();});
}

void PaintingWindow::pushObjectCanvas(){
    canvas->push_obj(&meshToPaint);
    meshToPaint.show_wireframe(false);
    meshToPaint.show_poly_color();

    compute_geodesics_amortized(meshToPaint, prefactored_matrices, {0});
    canvas->callback_mouse_press = [&](cinolib::GLcanvas *c, QMouseEvent *e)
    {
        cinolib::vec3d p;

        if (e->modifiers() == Qt::ControlModifier){

            if(c->unproject(cinolib::vec2i(e->x(),e->y()), p)) {
                createChart(p, true);
                meshToPaint.updateGL();
                c->updateGL();
            }
            //mergeCharts(currentChart);
        }

        if (e->modifiers() == Qt::ShiftModifier){

            if(c->unproject(cinolib::vec2i(e->x(),e->y()), p)) {
                createChart(p, false);
                meshToPaint.updateGL();
                c->updateGL();
            }
            //mergeCharts(currentChart);
        }
        return;
    };

    canvas->updateGL();

}

uint PaintingWindow::closest_vertex_paint(const cinolib::vec3d & p){
    std::vector<std::pair<double,uint>> closest;
    for(uint vid=0; vid<meshToPaint.num_verts(); ++vid)
        closest.push_back(std::make_pair(meshToPaint.vert(vid).dist(p),vid));
    std::sort(closest.begin(), closest.end());
    return closest.front().second;
}

std::string PaintingWindow::generateModelPath(bool openDocumentFolder,
                                               std::string meshName){
    cg3::viewer::LoaderSaver loaderSaverObj;
    std::string pathMesh;
    if(openDocumentFolder){
        loaderSaverObj.addSupportedExtension("obj");
        pathMesh = loaderSaverObj.loadDialog("Load Mesh");
    } else {
        pathMesh = "/home/toletto/Documenti/" + meshName;
    }

    return pathMesh;
}

void PaintingWindow::createChart(cinolib::vec3d p,
                                 bool paint){

    float brush_size = static_cast<float>(sl_size->value())/100.f;
    uint  vertexId = closest_vertex_paint(p);
    //std::set<uint> currentChart;
    cinolib::ScalarField f = compute_geodesics_amortized(meshToPaint, prefactored_matrices, {vertexId});

    for(uint faceId=0; faceId<meshToPaint.num_polys(); ++faceId)
    {

        //TODO Cercare il vertico più vicino del triangolo
        std::vector<float> closest;
        closest.push_back(f[meshToPaint.poly_vert_id(faceId,0)]);
        closest.push_back(f[meshToPaint.poly_vert_id(faceId,1)]);
        closest.push_back(f[meshToPaint.poly_vert_id(faceId,2)]);
        std::sort(closest.begin(), closest.end());
        float dist = 1.f - closest.front();
        //float dist = 1.f - f[meshToPaint.poly_vert_id(faceId,0)];
        if(dist<=brush_size)
        {
            //Gradiente
            //float val = meshToPaint.poly_data(faceId).color.g;
            //val -= (brush_size-dist)/brush_size;
            //if(val<0) val = 0.f;
            if(paint){
                meshToPaint.poly_data(faceId).color = cinolib::Color(1,0,0);
                paintedFaces[faceId] = paint;
            } else {
                meshToPaint.poly_data(faceId).color = cinolib::Color(1,1,1);
                paintedFaces[faceId] = paint;
            }
            //currentChart.insert(faceId);
        }
    }
}

void PaintingWindow::mergeCharts(std::set<uint>& currentChart){

    std::set<uint>::iterator aux;
    uint originalChart = 0;
    bool margeCharts = false;
    bool foundAnotherChart = false;
    for(uint chartId = 0; chartId < chartFaces.size(); chartId++){
        for(std::set<uint>::iterator it = currentChart.begin(); it != currentChart.end(); ++it){
            aux = chartFaces[chartId].find(*it);
            if(aux != chartFaces[chartId].end()){
                if(!foundAnotherChart){
                    for(std::set<uint>::iterator it = currentChart.begin(); it != currentChart.end(); ++it){
                        chartFaces[chartId].insert(*it);
                    }
                    margeCharts = true;
                    foundAnotherChart = true;
                    originalChart = chartId;
                    break;
                } else {
                    for(std::set<uint>::iterator it = chartFaces[chartId].begin(); it != chartFaces[chartId].end(); ++it){
                        chartFaces[originalChart].insert(*it);
                    }
                    chartFaces.erase(chartFaces.begin() + chartId);
                    break;
                }
            }
        }
    }
    if(!margeCharts){
        chartFaces.push_back(currentChart);
    }
}
