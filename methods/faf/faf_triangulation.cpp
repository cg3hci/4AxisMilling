#include "faf_triangulation.h"

namespace FourAxisFabrication {

namespace internal {

std::vector<std::array<cg3::Point2d, 3>> triangulation(
        std::vector<cg3::Point2d>& currentFirstLayerPoints2D,
        std::vector<cg3::Point2d>& newLayerPoints2D,
        std::set<cg3::Point2d>& newLayerPoints2DSet,
        std::vector<cg3::Point2d>& newLayerPoints2Dvector
        )
{
    //New layer vertices data
    //std::vector<cg3::Point2d> newLayerPoints2Dvector;
    std::vector<std::array<cg3::Point2d, 3>> triangulation;
    size_t nNewLayerVertices = newLayerPoints2D.size();
    size_t nFirstLayerVertices = currentFirstLayerPoints2D.size();

    //std::cout << "numero vertici chart: " << nFirstLayerVertices << std::endl;
    unsigned int idVertex = 0;

    //Add points into the set and map
    for (size_t i = 0; i < nNewLayerVertices; i++) {
        std::pair<std::set<cg3::Point2d>:: iterator, bool> it = newLayerPoints2DSet.insert(newLayerPoints2D[i]);
        if (it.second){
            newLayerPoints2Dvector.push_back(newLayerPoints2D[i]);
            idVertex++;
        }
    }

    //Triangulation
    unsigned int idNearestPointFinalBorder = 0, i=0, counter = 0, idPointChartBorder = 0;
    double minLengthVertex = std::numeric_limits<double>::max();

    //Iterator of vertex with minimun distance from first point of chart border
    for(i = 0; i< newLayerPoints2Dvector.size(); i++){
        cg3::Point2d ans = newLayerPoints2Dvector[i];
        cg3::Vec2d edgeVec = currentFirstLayerPoints2D[0] - ans;
        if(edgeVec.length() < minLengthVertex){
            idNearestPointFinalBorder = i;
            minLengthVertex = std::abs(edgeVec.length());
        }
    }

    while(idPointChartBorder <= nFirstLayerVertices){
        cg3::Point2d currentCharPoint = currentFirstLayerPoints2D[idPointChartBorder%nFirstLayerVertices];
        cg3::Point2d nextBorderPoint = newLayerPoints2Dvector[(idNearestPointFinalBorder+1)%idVertex];
        cg3::Point2d nextChartPoint = currentFirstLayerPoints2D[idPointChartBorder + 1];
        cg3::Point2d currentBorderPoint = newLayerPoints2Dvector[(idNearestPointFinalBorder)%idVertex];
        cg3::Vec2d edgeVecFromChart = currentCharPoint - nextBorderPoint;
        cg3::Vec2d edgeVecFromBorder = nextChartPoint - currentBorderPoint;

        if(idPointChartBorder == nFirstLayerVertices && counter == (idVertex-1)){
            std::array<cg3::Point2d, 3> vec = {currentCharPoint,
                                               currentBorderPoint,
                                               nextBorderPoint};
            triangulation.push_back(vec);
            break;
        }

        if(counter == idVertex && idPointChartBorder == (nFirstLayerVertices-1)){
            std::array<cg3::Point2d, 3> vec = {currentCharPoint,
                                               currentBorderPoint,
                                               nextChartPoint};
            triangulation.push_back(vec);
            break;
        }

        if((edgeVecFromChart.length() < edgeVecFromBorder.length()) && (counter < idVertex)){
            std::array<cg3::Point2d, 3> vec = {currentCharPoint,
                                               currentBorderPoint,
                                               nextBorderPoint};
            triangulation.push_back(vec);
            idNearestPointFinalBorder++;
            counter++;
        } else {
            std::array<cg3::Point2d, 3> vec = {currentCharPoint,
                                               currentBorderPoint,
                                               nextChartPoint};
            triangulation.push_back(vec);
            idPointChartBorder++;
        }
    }

    return triangulation;
}

void createSquare(std::vector<cg3::Point2d>& squarePoints2D, cg3::Point3d minCoord, cg3::Point3d maxCoord){

    unsigned int i=0;
    unsigned int nVertex = squarePoints2D.size();
    unsigned int nSideVertex = nVertex/4;
    unsigned int counterVertex = nSideVertex;
    double xStepLenght = (maxCoord.x() - minCoord.x())/nSideVertex;
    double yStepLenght = (maxCoord.y() - minCoord.y())/nSideVertex;

    for(; i < counterVertex; i++){
        squarePoints2D[i]=cg3::Point2d(minCoord.x()+(xStepLenght*(i%nSideVertex)), minCoord.y());
    }
    counterVertex += nSideVertex;

    for(; i<counterVertex; i++){
        squarePoints2D[i]=cg3::Point2d(maxCoord.x(), minCoord.y()+(yStepLenght*(i%nSideVertex)));
    }
    counterVertex += nSideVertex;

    for(; i < counterVertex; i++){
        squarePoints2D[i]=cg3::Point2d(maxCoord.x()-(xStepLenght*(i%nSideVertex)), maxCoord.y());
    }
    counterVertex += nSideVertex;

    for(; i < counterVertex; i++){
        squarePoints2D[i]=cg3::Point2d(minCoord.x(), maxCoord.y()-(yStepLenght*(i%nSideVertex)));
    }


}


} //Internal

} //FourAxisFabrication
