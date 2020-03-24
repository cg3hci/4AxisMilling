#include "paintingwindown.h"

PaintingWindow::PaintingWindow(std::vector<std::vector<size_t> > &value, cg3::DrawableEigenMesh& mesh) :
    partitions(value),
    drawablePaintedMesh(mesh)
{
    setWindow(false);

}

void PaintingWindow::setInstance(std::string meshName){
    if(meshName == "") loadEigenMesh();
    else loadMesh(meshName);
}

void PaintingWindow::showWindow(){
    window.show();
}

void PaintingWindow::setWindow(bool show){

    canvas.setParent(&window);
    but_reset.setText("Reset");
    but_reset.setParent(&window);
    but_next.setText("Confirm and close");
    but_next.setParent(&window);
    sl_size.setOrientation(Qt::Horizontal);
    sl_size.setParent(&window);
    sl_size.setMaximum(100);
    sl_size.setMinimum(0);
    sl_size.setValue(10);
    layout.addWidget(&sl_size);
    layout.addWidget(&but_reset);
    layout.addWidget(&but_next);
    layout.addWidget(&canvas);
    window.setLayout(&layout);
    if(show) window.show();
    window.resize(1024,1024);
}

void PaintingWindow::loadMesh(std::string meshName){
    meshToPaint = cinolib::DrawableTrimesh<>(meshName.c_str());
    pushObjectCanvas();

    connectButtons();

}

void PaintingWindow::loadEigenMesh(){

    meshToPaint.clear();
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
    QPushButton::connect(&but_reset, &QPushButton::clicked, [&]()
    {
        meshToPaint.poly_set_color( cinolib::Color::WHITE());

        canvas.updateGL();
    });

    QPushButton::connect(&but_next, &QPushButton::clicked, [&]()
    {

        partitions.clear();
        std::cout << chartFaces.size() << std::endl;
        for(uint chartId = 0; chartId < chartFaces.size(); chartId++){
            std::vector<size_t> currentChart;
            for(std::set<uint>::iterator it = chartFaces[chartId].begin(); it != chartFaces[chartId].end(); ++it){
                currentChart.push_back(*it);
            }
            partitions.push_back(currentChart);
        }

        canvas.updateGL();
        window.close();
    });


    // CMD+1 to show mesh controls.
    cinolib::SurfaceMeshControlPanel< cinolib::DrawableTrimesh<>> panel(&meshToPaint, &canvas);
    QApplication::connect(new QShortcut(QKeySequence(Qt::CTRL+Qt::Key_1), &canvas), &QShortcut::activated, [&](){panel.show();});
}

void PaintingWindow::pushObjectCanvas(){
    canvas.push_obj(&meshToPaint);
    meshToPaint.show_wireframe(false);
    meshToPaint.show_poly_color();

    compute_geodesics_amortized(meshToPaint, prefactored_matrices, {0});
    canvas.callback_mouse_press = [&](cinolib::GLcanvas *c, QMouseEvent *e)
    {
        if (e->modifiers() == Qt::ControlModifier)
        {
            cinolib::vec3d p;
            std::set<uint> currentChart;

            if(c->unproject(cinolib::vec2i(e->x(),e->y()), p))
            {
                float brush_size = static_cast<float>(sl_size.value())/100.f;
                uint  vertexId = closest_vertex_paint(p);

                cinolib::ScalarField f = compute_geodesics_amortized(meshToPaint, prefactored_matrices, {vertexId});

                createChartFromSeed(f,brush_size,currentChart);
                meshToPaint.updateGL();
                c->updateGL();
            }
            mergeCharts(currentChart);
        }
        return;
    };

    canvas.updateGL();

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

void PaintingWindow::createChartFromSeed(cinolib::ScalarField& f,
                                          float brush_size,
                                          std::set<uint>& currentChart){

    for(uint fid=0; fid<meshToPaint.num_polys(); ++fid)
    {

        //TODO Cercare il vertico più vicino del triangolo
        std::vector<float> closest;
        closest.push_back(f[meshToPaint.poly_vert_id(fid,0)]);
        closest.push_back(f[meshToPaint.poly_vert_id(fid,1)]);
        closest.push_back(f[meshToPaint.poly_vert_id(fid,2)]);
        std::sort(closest.begin(), closest.end());
        float dist = 1.f - closest.front();
        //float dist = 1.f - f[meshToPaint.poly_vert_id(fid,0)];
        if(dist<=brush_size)
        {
            //Gradiente
            //float val = meshToPaint.poly_data(fid).color.g;
            //val -= (brush_size-dist)/brush_size;
            //if(val<0) val = 0.f;
            meshToPaint.poly_data(fid).color = cinolib::Color(1,0,0);
            drawablePaintedMesh.setFaceColor(cg3::Color(255,0,0), fid);

            currentChart.insert(fid);
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

