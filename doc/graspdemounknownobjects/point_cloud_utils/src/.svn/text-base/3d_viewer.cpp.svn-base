#include <iostream>
#include <iomanip>
#include <fstream>
#include <ros/ros.h>
#include <GL/gl.h>
#include <GL/glut.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/point_cloud_conversion.h"
#include "Tuple3.hpp"
#include <cmath>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/ros/conversions.h"

using namespace std;

const GLfloat lightPosition[] = {-1, 1, 1, 0};
const GLfloat lightAmbient[]  = { 0.2, 0.2, 0.2, 0};
const GLfloat lightDiffuse[]  = { 0.8, 0.8, 0.8, 0};
const GLfloat lightSpecular[] = { 1, 1, 1, 0};

static const float ROTATE_SPEED     = 0.2f;
static const float TRANSLATE_SPEED  = 0.002f;
static const float CAM_INERTIA      = 0.1f;
static const float walkSpeed        = 0.045f;

static const int WIN_WIDTH  = 800;
static const int WIN_HEIGHT = 600;

static int mouseBtnState = 0;
static Tuple3f cameraRot(0, 0, 0), cameraRotLag(0, 0, 0);
static Tuple3f cameraPos(0, 1, 2), cameraPosLag(0, 1, 4);
static int lastMouseX, lastMouseY;
//static bool keyDown[256];
static GLdouble camViewMat[16];
static float scale = 1.0;
static bool showNormals = false;
static float normScale = 1.0;

static bool capturing = false;
static pcl::PointCloud<pcl::PointXYZRGBNormal> pcd_norm;
static pcl::PointCloud<pcl::PointXYZ> pcd, pcd_add, pcd_add2, pcd_temp;

template <typename T>
static inline void inverse_transform_vector(
        const Tuple3<T>* inv, Tuple3<T>* outv, const T* M)
{
    outv->x = inv->x*M[0] + inv->y*M[1] + inv->z*M[2];
    outv->y = inv->x*M[4] + inv->y*M[5] + inv->z*M[6];
    outv->z = inv->x*M[8] + inv->y*M[9] + inv->z*M[10];
}

template <typename T>
static inline void transform_point(
        const Tuple3<T>* inp, Tuple3<T>* outp, const T* M)
{
    outp->x = inp->x*M[0] + inp->y*M[4] + inp->z*M[8]  + M[12];
    outp->y = inp->x*M[1] + inp->y*M[5] + inp->z*M[9]  + M[13];
    outp->z = inp->x*M[2] + inp->y*M[6] + inp->z*M[10] + M[14];
}

template <typename T>
static inline void inverse_transform_point(
        const Tuple3<T>* inp, Tuple3<T>* outp, const T* M)
{
    Tuple3<T> aux = Tuple3<T>(inp->x - M[12], inp->y - M[13], inp->z - M[14]);
    inverse_transform_vector(&aux, outp, M);
}

/////////////////////////////////////////////////////////////////////////////////////////

static void point_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& pc)
{
    if ( !capturing ) return;
    if (showNormals) {
        pcl::fromROSMsg(*pc,pcd_norm);
        printf("size: %d of %d\n", pcd_norm.points.size(), pc->width*pc->height);
    } else {
        pcl::fromROSMsg(*pc,pcd);
        printf("size: %d of %d\n", pcd.points.size(), pc->width*pc->height);
    }
    //printf("size: %d\n", pcd.points.size());
}

static void point_add_callback(const sensor_msgs::PointCloud2::ConstPtr& pc)
{
    if ( !capturing ) return;
    pcl::fromROSMsg(*pc,pcd_add);
    printf("add size: %d of %d\n", pcd_add.points.size(), pc->width*pc->height);
}

static void point_add_callback2(const sensor_msgs::PointCloud2::ConstPtr& pc)
{
    if ( !capturing ) return;
    pcl::fromROSMsg(*pc,pcd_add2);
    printf("add size: %d of %d\n", pcd_add2.points.size(), pc->width*pc->height);
}

static void point_add_callback3(const sensor_msgs::PointCloud2::ConstPtr& pc)
{
    if ( !capturing ) return;
    pcl::fromROSMsg(*pc,pcd_temp);
    pcd_add2 += pcd_temp;
    printf("add size: %d of %d\n", pcd_add2.points.size(), pc->width*pc->height);
}

static void reshape(int w, int h)
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, (float)w / (float)h, 0.1f, 100.f);

    glMatrixMode(GL_MODELVIEW);
    glViewport(0, 0, w, h);
}

static void idle()
{
    ros::spinOnce();
    glutPostRedisplay();
}

static void display()
{
    // move camera
    cameraPosLag += (cameraPos - cameraPosLag) * CAM_INERTIA;
    cameraRotLag += (cameraRot - cameraRotLag) * CAM_INERTIA;

    // set up camera's view matrix
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glRotatef(cameraRotLag.x, 1.f, 0.f, 0.f);
    glRotatef(cameraRotLag.y, 0.f, 1.f, 0.f);
    glRotatef(-90.0f, 0.f, 0.f, 1.f);
    glTranslatef(-cameraPosLag.x, -cameraPosLag.y, -cameraPosLag.z);

    //// retrieve the current camera view matrix
    glGetDoublev(GL_MODELVIEW_MATRIX, camViewMat);

    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    glColor3f(1, 0, 0);

    glEnable(GL_LIGHTING);
    //glutSolidSphere(0.1, 24, 24);

    glDisable(GL_LIGHTING);
    if (showNormals) {
        glColor3f(0, 1, 0);
        glBegin(GL_POINTS);
        for(size_t i = 0;i < pcd_norm.points.size();++ i) {
            if (isnan(pcd_norm.points[i].x) || isnan(pcd_norm.points[i].y) || isnan(pcd_norm.points[i].z) ||
                isinf(pcd_norm.points[i].x) || isinf(pcd_norm.points[i].y) || isinf(pcd_norm.points[i].z)) {}
            else glVertex3f(pcd_norm.points[i].x*scale, pcd_norm.points[i].y*scale, pcd_norm.points[i].z*scale);
        }
        glEnd();
        glColor3f(1, 1, 1);
        glBegin(GL_LINES);
        for(size_t i = 0;i < pcd_norm.points.size();i+=20) {
            if (isnan(pcd_norm.points[i].x) || isnan(pcd_norm.points[i].y) || isnan(pcd_norm.points[i].z) ||
                isinf(pcd_norm.points[i].x) || isinf(pcd_norm.points[i].y) || isinf(pcd_norm.points[i].z)) {}
            else {
                glVertex3f(pcd_norm.points[i].x*scale, pcd_norm.points[i].y*scale, pcd_norm.points[i].z*scale);
                glVertex3f((pcd_norm.points[i].x+normScale*pcd_norm.points[i].normal[0])*scale,
                  (pcd_norm.points[i].y+normScale*pcd_norm.points[i].normal[1])*scale,
                  (pcd_norm.points[i].z+normScale*pcd_norm.points[i].normal[2])*scale);
            }
        }
        glEnd();
    } else {
        glColor3f(0, 1, 0);
        glBegin(GL_POINTS);
        for(size_t i = 0;i < pcd.points.size();++ i) {
            if (isnan(pcd.points[i].x) || isnan(pcd.points[i].y) || isnan(pcd.points[i].z) ||
                isinf(pcd.points[i].x) || isinf(pcd.points[i].y) || isinf(pcd.points[i].z)) {}
            else glVertex3f(pcd.points[i].x*scale, pcd.points[i].y*scale, pcd.points[i].z*scale);
        }
        glEnd();
    }
    
    glColor3f(0, 0, 1);
    glBegin(GL_POINTS);
    for(size_t i = 0;i < pcd_add.points.size();++ i) {
        if (isnan(pcd_add.points[i].x*scale) || isnan(pcd_add.points[i].y) || isnan(pcd_add.points[i].z) ||
            isinf(pcd_add.points[i].x) || isinf(pcd_add.points[i].y) || isinf(pcd_add.points[i].z)) {}
        else glVertex3f(pcd_add.points[i].x*scale, pcd_add.points[i].y*scale, pcd_add.points[i].z*scale);
    }
    glEnd();
    
    glColor3f(1, 0, 0);
    glBegin(GL_POINTS);
    for(size_t i = 0;i < pcd_add2.points.size();++ i) {
        if (isnan(pcd_add2.points[i].x) || isnan(pcd_add2.points[i].y) || isnan(pcd_add2.points[i].z) ||
            isinf(pcd_add2.points[i].x) || isinf(pcd_add2.points[i].y) || isinf(pcd_add2.points[i].z)) {}
        else glVertex3f(pcd_add2.points[i].x*scale, pcd_add2.points[i].y*scale, pcd_add2.points[i].z*scale);
    }
    glEnd();
    
    // Draw axis
    glBegin(GL_LINES);
    glColor3f(1, 0, 0);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(1.0f, 0.0f, 0.0f);
    glColor3f(0, 1, 0);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 1.0f, 0.0f);
    glColor3f(0, 0, 1);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 1.0f);
    glEnd();
    glutSwapBuffers();
}

/*
static void write_point_cloud(const char* fname)
{
  pcl::io::savePCDFileASCII ("pcd.pcd", pcd);
}
*/

static void key_handler(unsigned char key, int, int)
{
    //char fname[128];
    switch (key)
    {
        case ' ':
            capturing = !capturing;
            if ( capturing )
                printf("Start capturing\n");
            else
                printf("Stop capturing\n");
            break;
        case 'w':
            // cout << "Point Cloud File: ";
            // cin >> fname;
            // write_point_cloud(fname);
            break;
        case 'g':
            // write_point_cloud("pcd.txt");
            exit(0);
            break;
    }
}

static void mouse_button_handler(int button, int state, int x, int y)
{
    int mods;

    if ( state == GLUT_DOWN )
        mouseBtnState |= 1<<button;
    else if ( state == GLUT_UP )
        mouseBtnState = 0;

    mods = glutGetModifiers();
    if ( mods & GLUT_ACTIVE_SHIFT )
        mouseBtnState = 2;
    else if ( mods & GLUT_ACTIVE_CTRL )
        mouseBtnState = 3;

    lastMouseX = x; lastMouseY = y;

    //if ( showSlider ) pparams->Mouse(x, y, button, state);
    glutPostRedisplay();
}

static void mouse_motion_handler(int x, int y)
{
    ////// adjust slider
    //if ( showSlider && pparams->Motion(x, y) )
    //{
    //    lastMouseX = x; lastMouseY = y;
    //    glutPostRedisplay();
    //    return;
    //}

    Tuple3d v, r;
    float dx = (float)(x - lastMouseX);
    float dy = (float)(y - lastMouseY);

    switch ( mouseBtnState )
    {
        case 1:
            // rotate camera
            cameraRot.x += dy * ROTATE_SPEED;
            cameraRot.y += dx * ROTATE_SPEED;
            break;
        case 2:
            // translate camera
            v.set(dx*TRANSLATE_SPEED, -dy*TRANSLATE_SPEED, 0.f);
            inverse_transform_vector(&v, &r, camViewMat);
            cameraPos -= r;
            break;
        case 3:
            // zoom
            v.set(0.f, 0.f, dy*TRANSLATE_SPEED);
            inverse_transform_vector(&v, &r, camViewMat);
            cameraPos += r;
            break;
    }

    lastMouseX = x; lastMouseY = y;
    glutPostRedisplay();
}

static void init_opengl()
{
    int dummy = 0;
    glutInit(&dummy, NULL);

    glutInitDisplayMode(GLUT_RGBA|GLUT_DOUBLE|GLUT_DEPTH);
    glutInitWindowSize(WIN_WIDTH, WIN_HEIGHT);
    glutCreateWindow("bumblebee point cloud data");

    glClearColor(0, 0, 0, 0);
    glClearDepth(1);
    glEnable(GL_DEPTH_TEST);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_LINE_SMOOTH);

    // lighting
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
    glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
}

int main(int argc, char* argv[])
{
    vector<char*> srcs;
    if (argc <= 1) {
        srcs.push_back("/bumblebee2/points2");
    } else {
        int i = 1; bool isDone = false;
        while (!isDone) {
            if (strcmp(argv[i],"-scale") == 0) {
                scale = atof(argv[i+1]);
                i += 2;
            } else if (strcmp(argv[i],"-n") == 0) {
                showNormals = true;
                normScale = atof(argv[i+1]);
                i += 2;
            } else {
                isDone = true;
            }
        }
        for (;i<argc;i++) {
            srcs.push_back(argv[i]);
        }
    }
    for (unsigned int i=0;i<srcs.size();i++)
        printf("%s\n",srcs[i]);
    printf("showNormals = %s\n normScale = %f\n, scale = %f\n",
        (showNormals?"true":"false"),
        normScale,
        scale);

    // setup opengl
    init_opengl();
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(key_handler);
    //glutKeyboardUpFunc(key_up_handler);
    glutMouseFunc(mouse_button_handler);
    glutMotionFunc(mouse_motion_handler);
    glutIdleFunc(idle);

    // setup subscriber
    ros::init(argc, argv, "guiviewer", ros::init_options::AnonymousName);
    ros::NodeHandle nd;
    vector<ros::Subscriber> pcdSub;
    pcdSub.push_back(nd.subscribe(srcs[0], 1, point_cloud_callback));
    if (srcs.size() >= 2)
        pcdSub.push_back(nd.subscribe(srcs[1], 1, point_add_callback));
    if (srcs.size() >= 3)
        pcdSub.push_back(nd.subscribe(srcs[2], 1, point_add_callback2));
    for (unsigned int i=3;i<srcs.size();i++)
        pcdSub.push_back(nd.subscribe(srcs[i], 1, point_add_callback3));
    /*ros::Subscriber pcdSub;
    for (int i=0;i<srcs.size();i++)
        pcdSub = (nd.subscribe(srcs[i], 5, point_add_callback));
    */
    capturing = true;
    // clean up
    glutMainLoop();

    return 0;
}
