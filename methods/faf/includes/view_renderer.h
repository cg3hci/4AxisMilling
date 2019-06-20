#ifndef VIEWRENDERER_H
#define VIEWRENDERER_H

#include <cg3/geometry/bounding_box3.h>
#include <cg3/meshes/eigenmesh/eigenmesh.h>

#include <unordered_set>

#include <QOpenGLFunctions_4_3_Core>
#include <QOffscreenSurface>
#include <QOpenGLFramebufferObject>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLShaderProgram>
#include <QMatrix4x4>



class ViewRenderer
{
public:
    ViewRenderer(const cg3::SimpleEigenMesh& mesh, int resolution = 2048);
    ViewRenderer(const cg3::SimpleEigenMesh& mesh, const cg3::BoundingBox3& bb, int resolution = 2048);
    ~ViewRenderer();

    std::vector<bool> renderVisibility(const cg3::Vec3& dir, bool exact = true, bool saveImg = false);
    void resetFaces();
    void addFaces(const std::vector<unsigned int>& faces);
    void removeLastFaces(unsigned int n);

private:
    //opengl
    QOffscreenSurface offscreenSurface;
    QOpenGLContext openglContext; //opengl context
    QRect imgRectangle; //rectangle of the image

    QOpenGLShaderProgram* visibilityShader; //visibility shader
    QOpenGLShaderProgram* occlusionShader; //occlusion shader
    QOpenGLFunctions_4_3_Core* openglf; //opengl context functions

    GLuint vertexArrayObjectID, vertexArrayObjectQuadID;
    GLuint frameBufferObjectID1, frameBufferObjectID2;
    GLuint m_indexAtt;
    GLuint m_depthAtt;
    GLuint shaderBufferOccludedFacesID;

    //Mesh
    cg3::SimpleEigenMesh mesh;
    cg3::BoundingBox3 boundingBox;

    std::vector<GLfloat> verticesArray;
    std::vector<GLint> vertexToFaceIndicesArray;
    std::vector<GLuint> faceIndicesArray;
    unsigned int numRenderedFaces;

    void initializeOpenglContext(int resolution);

    void setUpOpenGLContext();
    void loadShader(QOpenGLShaderProgram*& shader, QOffscreenSurface& surf, const char* vert, const char* frag);
    void initVao();
    void initVertAndIndices();

    //visibility functions
    QMatrix4x4 computeOrtho();
    std::vector<bool> computeVisibleFacets(const QMatrix4x4& mvp);
    void removePartiallyOccludedFacets(std::vector<bool>& visible, const QMatrix4x4& mvp);
    void saveImage(const cg3::Vec3& dir);
};

#endif // VIEWRENDERER_H
