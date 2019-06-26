#include "view_renderer.h"

#include <QOpenGLBuffer>
#include <QImage>
#include <QDebug>

ViewRenderer::ViewRenderer(const cg3::SimpleEigenMesh& m, int resolution) :
    ViewRenderer(m, m.boundingBox(), resolution)
{
}

ViewRenderer::ViewRenderer(
        const cg3::SimpleEigenMesh& m,
        const cg3::BoundingBox3& bb,
        int resolution) :
    mesh(m)
{
    initializeOpenglContext(resolution);

    boundingBox = bb;
    //load verts and indices for original mesh
    initVertAndIndices();
    setUpOpenGLContext();
}

ViewRenderer::~ViewRenderer()
{
    openglContext.makeCurrent(&offscreenSurface);
    openglf->glDeleteFramebuffers(1, &frameBufferObjectID1);
    openglf->glDeleteFramebuffers(1, &frameBufferObjectID2);

	//	delete m_fbo;
	//delete vaos and vbos etc..
    openglContext.doneCurrent();
}

std::vector<bool> ViewRenderer::renderVisibility(const cg3::Vec3d &dir, bool exact, bool saveImg)
{
    QQuaternion q = QQuaternion::rotationTo(QVector3D(dir[0], dir[1], dir[2]), QVector3D(0,0,1));
    QMatrix4x4 mv(q.toRotationMatrix());
    QMatrix4x4 p = computeOrtho();
    QMatrix4x4 mvp = p * mv;



    //bind fbo1 and clear
    openglf->glBindFramebuffer(GL_FRAMEBUFFER, frameBufferObjectID1);
    openglf->glViewport(0, 0, imgRectangle.width(), imgRectangle.height());
    openglf->glClearColor(0, 0, 0, 0);
    openglf->glDrawBuffer(GL_COLOR_ATTACHMENT0);
    openglf->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    std::vector<bool> visible = computeVisibleFacets(mvp);

    if (saveImg){
        saveImage(dir);
    }

    //be nazi: discard all indices s.t. at least one fragment is occluded
    /* disable depth and cull to get all faces */
    /* draw all face indices of frags behind something */
    /* clear occlusion buffer */

    if (exact){
        removePartiallyOccludedFacets(visible, mvp);
    }

    openglf->glBindFramebuffer(GL_FRAMEBUFFER, 0);

    return visible;
}

void ViewRenderer::resetFaces()
{
    faceIndicesArray.clear();
    numRenderedFaces = 0;
    /*openglf->glBindVertexArray(vertexArrayObjectID);
    openglf->glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, 0, (void*)0);
    openglf->glBindVertexArray(0);*/

}

void ViewRenderer::addFaces(const std::vector<unsigned int>& faces)
{
    for (unsigned int f : faces){
        faceIndicesArray.push_back(f*3);
        faceIndicesArray.push_back(f*3+1);
        faceIndicesArray.push_back(f*3+2);
    }
    numRenderedFaces += faces.size();
    openglf->glBindVertexArray(vertexArrayObjectID);
    openglf->glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, numRenderedFaces*3 * sizeof(GLuint), faceIndicesArray.data());
    //openglf->glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, numRenderedFaces*3 * sizeof(GLuint),
    //                         faces.size() * 3 * sizeof(GLuint), &(faceIndicesArray[numRenderedFaces]));
    openglf->glBindVertexArray(0);

}

void ViewRenderer::removeLastFaces(unsigned int n)
{
    assert(n <= numRenderedFaces);
    faceIndicesArray.resize(faceIndicesArray.size() - n*3);
    numRenderedFaces-=n;
}

void ViewRenderer::initializeOpenglContext(int resolution)
{
    QSurfaceFormat fmt;
    fmt.setRenderableType(QSurfaceFormat::OpenGL);
    fmt.setVersion(4,3);
    fmt.setProfile(QSurfaceFormat::CoreProfile);

    offscreenSurface.setFormat(fmt);
    offscreenSurface.create();
    if(!offscreenSurface.isValid())
        qFatal("Error creating offscreen surface");

    if(!openglContext.create())
        qFatal("Error creating opengl context");

    openglContext.setFormat(fmt);
    openglContext.makeCurrent(&offscreenSurface);

    openglf = openglContext.versionFunctions<QOpenGLFunctions_4_3_Core>();
    openglf->initializeOpenGLFunctions();

    GLint dims[2];
    openglf->glGetIntegerv(GL_MAX_VIEWPORT_DIMS, &dims[0]);
    imgRectangle = QRect(0, 0, resolution, resolution);
    //std::cout << dims[0] << ":" << dims[1] << std::endl;
}

void ViewRenderer::setUpOpenGLContext()
{
    //init vao
    initVao();

    GLfloat quadVerts[] = {
        -1.0,  1.0,
        -1.0, -1.0,
        1.0, -1.0,

        -1.0,  1.0,
        1.0, -1.0,
        1.0,  1.0
    };
    openglf->glGenVertexArrays(1, &vertexArrayObjectQuadID);
    openglf->glBindVertexArray(vertexArrayObjectQuadID);
    GLuint vbo;
    openglf->glGenBuffers(1, &vbo);
    openglf->glBindBuffer(GL_ARRAY_BUFFER, vbo);
    openglf->glBufferData(GL_ARRAY_BUFFER, sizeof(quadVerts), quadVerts, GL_STATIC_DRAW);
    openglf->glEnableVertexAttribArray(0);
    openglf->glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(GLfloat), (void*)0);
    openglf->glBindBuffer(GL_ARRAY_BUFFER, 0);
    openglf->glBindVertexArray(0);

    loadShader(visibilityShader, offscreenSurface, ":/visible.vert", ":/visible.frag");
    loadShader(occlusionShader, offscreenSurface, ":/occluded.vert", ":/occluded.frag");

    /* setup the shader buffer object to be used as 1D buffer to save occluded indices */
    //GLuint* m_data = new GLuint[m.getNumberFaces()];
    std::vector<GLuint> m_data;
    m_data.resize(mesh.numberFaces());
    openglf->glGenBuffers(1, &shaderBufferOccludedFacesID);
    openglf->glBindBuffer(GL_SHADER_STORAGE_BUFFER, shaderBufferOccludedFacesID);
    openglf->glBufferData(GL_SHADER_STORAGE_BUFFER, mesh.numberFaces() * sizeof(GLuint), m_data.data(), GL_DYNAMIC_COPY);
    openglf->glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, shaderBufferOccludedFacesID);
    openglf->glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    /* setup the framebuffer with 32F color_att and depth_text used to render depth */
    openglf->glGenFramebuffers(1, &frameBufferObjectID1);
    openglf->glBindFramebuffer(GL_FRAMEBUFFER, frameBufferObjectID1);

    openglf->glGenTextures(1, &m_indexAtt);
    openglf->glBindTexture(GL_TEXTURE_2D, m_indexAtt);
    openglf->glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, imgRectangle.width(), imgRectangle.height(), 0, GL_RED, GL_FLOAT, NULL);
    openglf->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    openglf->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    openglf->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_indexAtt, 0);
    openglf->glBindTexture(GL_TEXTURE_2D, 0);

    openglf->glGenTextures(1, &m_depthAtt);
    openglf->glBindTexture(GL_TEXTURE_2D, m_depthAtt);
    openglf->glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, imgRectangle.width(), imgRectangle.height(), 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    openglf->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    openglf->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    openglf->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, m_depthAtt, 0);
    openglf->glBindTexture(GL_TEXTURE_2D, 0);

    occlusionShader->bind();
    occlusionShader->setUniformValue("depthmap", 0);
    occlusionShader->release();

    if(!(openglf->glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE)) //modified
    {
        qDebug() << "[ERROR] Setting up the FBO";
        abort();
    }
    openglf->glBindFramebuffer(GL_FRAMEBUFFER, 0);


    //setup second framebuffer
    openglf->glGenFramebuffers(1, &frameBufferObjectID2);
    openglf->glBindFramebuffer(GL_FRAMEBUFFER, frameBufferObjectID2);

    openglf->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_indexAtt, 0);

    openglf->glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void ViewRenderer::loadShader(QOpenGLShaderProgram* &shader, QOffscreenSurface &surf, const char* vert, const char* frag)
{
    //init shader program
    shader = new QOpenGLShaderProgram(&surf);
    bool vOK = shader->addShaderFromSourceFile(QOpenGLShader::Vertex,   vert);
    bool fOK = shader->addShaderFromSourceFile(QOpenGLShader::Fragment, frag);
    bool lOK = shader->link();

    if(!(vOK && fOK && lOK))
    {
        qDebug() << shader->log();
        abort();
    }
}

void ViewRenderer::initVao()
{
    //init vao and vbo
    openglf->glGenVertexArrays(1, &vertexArrayObjectID); // saves in vao[0] the name of the generated array
    openglf->glBindVertexArray(vertexArrayObjectID); //binds the vao, from now on the calls are done in this vao

    /* setup the vertex buffer object containing the face indices */
    GLuint vbo;
    openglf->glGenBuffers(1, &vbo);
    openglf->glBindBuffer(GL_ARRAY_BUFFER, vbo);
    openglf->glBufferData(GL_ARRAY_BUFFER,  verticesArray.size() * sizeof(GLfloat), verticesArray.data(), GL_STATIC_DRAW);
    openglf->glEnableVertexAttribArray(1);
    openglf->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (void*) 0);
    openglf->glBindBuffer(GL_ARRAY_BUFFER, 0);

    /* setup the vertex buffer object containing the vertex indices of every face */
    GLuint ebo;
    openglf->glGenBuffers(1, &ebo);
    openglf->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    openglf->glBufferData(GL_ELEMENT_ARRAY_BUFFER, faceIndicesArray.size() * sizeof(GLuint), faceIndicesArray.data(), GL_STATIC_DRAW);

    /* setup the vertex buffer object containing the face indices for every vertex */
    GLuint ibo;
    openglf->glGenBuffers(1, &ibo);
    openglf->glBindBuffer(GL_ARRAY_BUFFER, ibo);
    openglf->glBufferData(GL_ARRAY_BUFFER, vertexToFaceIndicesArray.size() * sizeof(GLint), vertexToFaceIndicesArray.data(), GL_STATIC_DRAW);
    openglf->glEnableVertexAttribArray(0);
    openglf->glVertexAttribIPointer(0, 1, GL_INT, sizeof(GLint), (void*) 0);
    openglf->glBindBuffer(GL_ARRAY_BUFFER, 0);

    openglf->glBindVertexArray(0); //closes the binding of m_vao.....
}

void ViewRenderer::initVertAndIndices()
{
    verticesArray.resize(mesh.numberFaces()* 3 * 3);
    vertexToFaceIndicesArray.resize(mesh.numberFaces() * 3);
    faceIndicesArray.resize(mesh.numberFaces() * 3);
    int k = 0, j = 0;
    for (unsigned int fi = 0; fi < mesh.numberFaces(); fi++){
        cg3::Point3i f = mesh.face(fi);
        for (unsigned int vi = 0; vi < 3; vi++){
            //jth vertex...
            verticesArray[k++] = (GLfloat) mesh.vertex(f[vi]).x();
            verticesArray[k++] = (GLfloat) mesh.vertex(f[vi]).y();
            verticesArray[k++] = (GLfloat) mesh.vertex(f[vi]).z();

            vertexToFaceIndicesArray[j++] = (GLint) fi; //fi doesn't change
        }
    }
    for (unsigned int i = 0; i < faceIndicesArray.size(); i++)
        faceIndicesArray[i] = (GLuint)i;
    numRenderedFaces = mesh.numberFaces();
}

QMatrix4x4 ViewRenderer::computeOrtho()
{
    QMatrix4x4 p;
    double radius = boundingBox.diag()/2;
    p.ortho(-radius, radius, -radius, radius, -radius, radius);
    return p;
}

std::vector<bool> ViewRenderer::computeVisibleFacets(const QMatrix4x4& mvp)
{
    //init visibility set (foreach face stores visibility info)
    //m_numvert is m.fn() * 3..
    std::vector<bool> visible(mesh.numberFaces(), false);

    /* draw depth & visibile faces */

    openglf->glEnable(GL_DEPTH_TEST);
    openglf->glDepthFunc(GL_LESS);
    openglf->glEnable(GL_CULL_FACE);

    visibilityShader->bind();
    visibilityShader->setUniformValue("mvp", mvp);

    //compute visibility..
    openglf->glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

    openglf->glBindVertexArray(vertexArrayObjectID);

    openglf->glDrawElements(GL_TRIANGLES, faceIndicesArray.size(), GL_UNSIGNED_INT, (void*)0);

    openglf->glFinish();

    //store direct visibility
    //qDebug() << "Storing computed visibility";
    openglf->glBindTexture(GL_TEXTURE_2D, m_indexAtt);

    const int size = imgRectangle.width() * imgRectangle.height();
    std::vector<GLfloat> pix(size);
    openglf->glGetTexImage(GL_TEXTURE_2D, 0, GL_RED, GL_FLOAT, (GLvoid*) pix.data());

    //setting visibility for each face s.t. at least one fragment is visible...
    for(int i = 0; i < size; ++i){
        if (pix[i] > 0 && pix[i] <= mesh.numberFaces()){
            visible[pix[i] - 1] = true;
        }
    }
    openglf->glBindTexture(GL_TEXTURE_2D, 0);
    openglf->glBindVertexArray(0);
    return visible;
}

void ViewRenderer::removePartiallyOccludedFacets(std::vector<bool>& visible, const QMatrix4x4& mvp)
{
    openglf->glBindVertexArray(vertexArrayObjectID);
    openglf->glBindFramebuffer(GL_FRAMEBUFFER, frameBufferObjectID2);
    openglf->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    /* bind the storage buffer and clear it*/
    openglf->glBindBuffer(GL_SHADER_STORAGE_BUFFER, shaderBufferOccludedFacesID);
    GLuint clearV = 0;
    openglf->glClearBufferData(GL_SHADER_STORAGE_BUFFER, GL_R32UI, GL_RED, GL_UNSIGNED_INT, &clearV);
    openglf->glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    /* bind the occluded fragment drawing shader */
    occlusionShader->bind();
    occlusionShader->setUniformValue("mvp", mvp);

    /* bind the depth texture from the previous pass */
    openglf->glActiveTexture(GL_TEXTURE0);
    openglf->glBindTexture(GL_TEXTURE_2D, m_depthAtt);

    /* draw face indexes s.t. at least one frag is occluded */
    openglf->glDisable(GL_DEPTH_TEST);
    openglf->glDepthFunc(GL_ALWAYS);
    openglf->glDisable(GL_CULL_FACE);
    openglf->glDrawElements(GL_TRIANGLES, faceIndicesArray.size(), GL_UNSIGNED_INT, (void*)0);
    //openglf->glDrawArrays(GL_TRIANGLES, 0, verticesArray.size()); //??????

    openglf->glBindTexture(GL_TEXTURE_2D, 0);

    openglf->glFinish();

    openglf->glBindBuffer(GL_SHADER_STORAGE_BUFFER, shaderBufferOccludedFacesID);
    GLuint* data = (GLuint*) openglf->glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, mesh.numberFaces() * sizeof(GLuint), GL_MAP_READ_BIT);

    //drop visibility for each occluded face

    for (unsigned int i = 0; i < mesh.numberFaces(); ++i)
        if (data[i] == 1)
            visible[i] = false;


    openglf->glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);

    occlusionShader->release();
    openglf->glBindVertexArray(0);
    openglf->glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void ViewRenderer::saveImage(const cg3::Vec3d &dir)
{
    openglf->glBindTexture(GL_TEXTURE_2D, m_indexAtt);

    std::vector<GLfloat> pix(imgRectangle.width()*imgRectangle.width());
    //GLfloat* pix = new GLfloat[m_rect.width()*m_rect.width()];
    openglf->glGetTexImage(GL_TEXTURE_2D, 0, GL_RED, GL_FLOAT, pix.data());
    QImage img((uchar*) pix.data(), imgRectangle.width(), imgRectangle.width(), QImage::Format_RGB32);
    {
        std::ostringstream bp;
        bp << "red_" << dir[0] << "_" << dir[1] << "_" << dir[2] << ".png";
        img.save(bp.str().c_str());
    }
    openglf->glBindTexture(GL_TEXTURE_2D, 0);

    openglf->glBindTexture(GL_TEXTURE_2D, m_depthAtt);

    std::vector<GLfloat> pix0(imgRectangle.width()*imgRectangle.width());
    //GLfloat* pix0 = new GLfloat[m_rect.width()*m_rect.width()];
    openglf->glGetTexImage(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, GL_FLOAT, pix0.data());
    QImage img0((uchar*) pix0.data(), imgRectangle.width(), imgRectangle.width(), QImage::Format_RGB32);
    qDebug() << img0.format();
    {
        std::ostringstream bp;
        bp << "depth_" << dir[0] << "_" << dir[1] << "_" << dir[2] << ".png";
        img0.save(bp.str().c_str());
    }
    openglf->glBindTexture(GL_TEXTURE_2D, 0);
}
