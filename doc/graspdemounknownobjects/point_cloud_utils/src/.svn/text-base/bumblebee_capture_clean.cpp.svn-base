#include <iostream>
#include <iomanip>
#include <fstream>
#include <ros/ros.h>
#include <GL/gl.h>
#include <GL/glut.h>
#include <sensor_msgs/PointCloud.h>
#include "Tuple3.hpp"

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
static Tuple3f cameraPos(0, 1, 4), cameraPosLag(0, 1, 4);
static int lastMouseX, lastMouseY;
//static bool keyDown[256];
static GLdouble camViewMat[16];

static bool capturing = false;
static vector<geometry_msgs::Point32>  pcd;

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
    Tuple3<T> aux(inp->x - M[12], inp->y - M[13], inp->z - M[14]);
    inverse_transform_vector(&aux, outp, M);
}

/////////////////////////////////////////////////////////////////////////////////////////

static void transform_to_global_frame(vector<geometry_msgs::Point32> &pcd)
{
    double trans[3][4] = {{48.19820571, -242.16136401, 37.8146079, 407.39378972},
 {-241.56135225,  -39.0242279,    47.94096509, -156.21965005},
 { -42.95519906,  -44.66765686, -241.58229251,  767.63208745}};

    for (int i=0; i<pcd.size(); i++)
    {
        double x = pcd[i].x, y = pcd[i].y, z = pcd[i].z;
        pcd[i].x = trans[0][0]*x + trans[0][1]*y+trans[0][2]*z+trans[0][3];
        pcd[i].y = trans[1][0]*x + trans[1][1]*y+trans[1][2]*z+trans[1][3];
        pcd[i].z = trans[2][0]*x + trans[2][1]*y+trans[2][2]*z+trans[2][3];    
    }

}
static void clean_missing_point(const sensor_msgs::PointCloud::ConstPtr& data)
{
    pcd.clear();
    //get maximum z
    double maxz=-1e10, eps=1e-6;
    for (int i=0; i<data->points.size(); i++)
        if (maxz<data->points[i].z)
            maxz = data->points[i].z;
    //remove all points beyond this threshold
    for (int i=0; i<data->points.size(); i++)
        if (data->points[i].z<maxz-eps)
            pcd.push_back(data->points[i]);    
    
}
static void point_cloud_callback(const sensor_msgs::PointCloud::ConstPtr& data)
{
    if ( !capturing ) return;

    printf("size: %d\n", data->get_points_size());
    //if ( data->get_points_size() != pcd.size() )
    //{
    //    fprintf(stderr, "point size is inconsisitent %d vs %d\n", 
    //            (int)data->get_points_size(), (int)pcd.size());
    //    return;
    //}

    //discard all missing point and store the rest in pcd
    clean_missing_point(data);
    transform_to_global_frame(pcd);
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
    glTranslatef(-cameraPosLag.x, -cameraPosLag.y, -cameraPosLag.z);

    //// retrieve the current camera view matrix
    glGetDoublev(GL_MODELVIEW_MATRIX, camViewMat);

    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    glColor3f(1, 0, 0);

    glEnable(GL_LIGHTING);
    glutSolidSphere(0.1, 24, 24);

    glDisable(GL_LIGHTING);
    glColor3f(0, 1, 0);
    glBegin(GL_POINTS);
    for(size_t i = 0;i < pcd.size();++ i)
        glVertex3f(pcd[i].x/10, pcd[i].y/10, pcd[i].z/10);
    glEnd();
    glutSwapBuffers();
}

static void write_point_cloud(const char* fname)
{
    ofstream fout(fname);
    fout << setprecision(20);
    fout << pcd.size() << endl;
    for(size_t i = 0;i < pcd.size();++ i)
        fout << pcd[i].x << ' ' << pcd[i].y << ' ' << pcd[i].z << endl;
    fout.close();
}

static void key_handler(unsigned char key, int, int)
{
    char fname[128];
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
            cout << "Point Cloud File: ";
            cin >> fname;
            write_point_cloud(fname);
            break;
        case 'g':
            write_point_cloud("pcd_clean.txt");
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
    pcd.clear();
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
    ros::init(argc, argv, "bumblebee_capture");
    ros::NodeHandle nd;
    ros::Subscriber pcdSub = nd.subscribe("/bumblebee2/points", 5, point_cloud_callback);
    // clean up
    glutMainLoop();

    return 0;
}
