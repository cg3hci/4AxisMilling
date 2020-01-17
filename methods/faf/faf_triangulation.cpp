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

    unsigned int numBorderVertex = 0;

    //Add points into the set and map
    for (size_t i = 0; i < nNewLayerVertices; i++) {
        std::pair<std::set<cg3::Point2d>:: iterator, bool> it = newLayerPoints2DSet.insert(newLayerPoints2D[i]);
        if (it.second){
            newLayerPoints2Dvector.push_back(newLayerPoints2D[i]);
            numBorderVertex++;
        }
    }

    //Triangulation
    unsigned int currentBorderVertexID = 0, i = 0, counterBorderVertexID = 0, currentChartVertexID = 0;
    double minLengthVertex = std::numeric_limits<double>::max();

    //Iterator of vertex with minimun distance from first point of chart border
    for(i = 0; i< newLayerPoints2Dvector.size(); i++){
        cg3::Point2d ans = newLayerPoints2Dvector[i];
        cg3::Vec2d edgeVec = currentFirstLayerPoints2D[0] - ans;
        if(edgeVec.length() < minLengthVertex){
            currentBorderVertexID = i;
            minLengthVertex = std::abs(edgeVec.length());
        }
    }
    while(currentChartVertexID <= nFirstLayerVertices){
        cg3::Point2d currentCharPoint = currentFirstLayerPoints2D[currentChartVertexID%nFirstLayerVertices];
        cg3::Point2d nextBorderPoint = newLayerPoints2Dvector[(currentBorderVertexID + 1) % numBorderVertex];
        cg3::Point2d nextChartPoint = currentFirstLayerPoints2D[(currentChartVertexID + 1) % nFirstLayerVertices];
        cg3::Point2d currentBorderPoint = newLayerPoints2Dvector[(currentBorderVertexID)%numBorderVertex];
        cg3::Vec2d edgeVecFromChart = currentCharPoint - nextBorderPoint;
        cg3::Vec2d edgeVecFromBorder = nextChartPoint - currentBorderPoint;
        cg3::Vec2d currentChartVec = currentCharPoint;
        cg3::Vec2d currentBorderVec = currentBorderPoint;
        cg3::Vec2d nextBorderVec = nextBorderPoint;

        if(currentChartVertexID == nFirstLayerVertices && counterBorderVertexID == (numBorderVertex - 1)){
            std::array<cg3::Point2d, 3> vec = {currentCharPoint,
                                               currentBorderPoint,
                                               nextBorderPoint};
            triangulation.push_back(vec);
            break;
        }

        if(counterBorderVertexID == numBorderVertex && currentChartVertexID == (nFirstLayerVertices - 1)){
            std::array<cg3::Point2d, 3> vec = {currentCharPoint,
                                               currentBorderPoint,
                                               nextChartPoint};
            triangulation.push_back(vec);
            break;
        }


        if((edgeVecFromChart.length() < edgeVecFromBorder.length()) && (counterBorderVertexID < numBorderVertex) /*&&
                currentChartVec.dot(nextBorderPoint) <= currentChartVec.dot(currentBorderVec)*/){
            //std::cout << "Dot chart and current border: " << currentChartVec.dot(currentBorderVec) << std::endl;
            //std::cout << "Dot chart and next border: " << currentChartVec.dot(nextBorderPoint) << std::endl;

            std::array<cg3::Point2d, 3> vec = {currentCharPoint,
                                               currentBorderPoint,
                                               nextBorderPoint};
            triangulation.push_back(vec);
            currentBorderVertexID++;
            counterBorderVertexID++;
        } else {
            std::array<cg3::Point2d, 3> vec = {currentCharPoint,
                                               currentBorderPoint,
                                               nextChartPoint};
            triangulation.push_back(vec);
            currentChartVertexID++;
        }
    }

    return triangulation;
}

void createSquare(std::vector<cg3::Point2d>& squarePoints2D, cg3::Point3d minCoord, cg3::Point3d maxCoord){

    unsigned int i = 0;
    unsigned int nVertex = squarePoints2D.size();
    unsigned int nSideVertex = nVertex / 4;
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

double computeHeight(
                     cg3::Point2d vertex,
                     std::map<cg3::Point2d, unsigned int>& currentFirstLayerPoints2DMap,
                     Eigen::Matrix3d projectionMatrix,
                     cg3::EigenMesh& result,
                     std::vector<cg3::Point2d>& currentFirstLayerPoints2D,
                     const double currentStepHeight){

    std::vector<cg3::Point2d>::iterator it = std::find(currentFirstLayerPoints2D.begin(), currentFirstLayerPoints2D.end(), vertex);
    unsigned int pos = distance(currentFirstLayerPoints2D.begin(), it);
    double height;

    cg3::Point3d p3Dprevprev = result.vertex(currentFirstLayerPoints2DMap.at(currentFirstLayerPoints2D[(pos - 2) % currentFirstLayerPoints2DMap.size()]));
    cg3::Point3d p3Dprev = result.vertex(currentFirstLayerPoints2DMap.at(currentFirstLayerPoints2D[(pos - 1) % currentFirstLayerPoints2DMap.size()]));
    cg3::Point3d p3D = result.vertex(currentFirstLayerPoints2DMap.at(vertex));
    cg3::Point3d p3Dnext = result.vertex(currentFirstLayerPoints2DMap.at(currentFirstLayerPoints2D[(pos + 1) % currentFirstLayerPoints2DMap.size()]));
    cg3::Point3d p3Dnextnext = result.vertex(currentFirstLayerPoints2DMap.at(currentFirstLayerPoints2D[(pos + 2) % currentFirstLayerPoints2DMap.size()]));
    p3Dprevprev.rotate(projectionMatrix);
    p3Dprev.rotate(projectionMatrix);
    p3D.rotate(projectionMatrix);
    p3Dnext.rotate(projectionMatrix);
    p3Dnextnext.rotate(projectionMatrix);

    height = (((p3Dnext.z() + p3Dnextnext.z() + p3D.z() + p3Dprev.z() + p3Dprevprev.z()) / 5) + currentStepHeight);

    return height;
}

} //Internal

} //FourAxisFabrication
