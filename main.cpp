#ifdef __APPLE_CC__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include "Mesh.h"

int gridX = 600;
int gridY = 600;
int gridZ = 600;

const double fovy = 50.;
const double clipNear = .01;
const double clipFar = 1000.;
double x = 0.0, y = 0.0, z = 0.0;
double eyeX = 0.0, eyeY = 1.0, eyeZ = 2.5; // camera points initially along y-axis
double upX = 0.0, upY = 1.0, upZ = 0.0; // camera points initially along y-axis
double r = 2.5, theta = 0.0, phi = 0.0;

std::vector<std::string> paths = {"/Users/rohansawhney/Desktop/developer/C++/model-repair/cow.obj",
                                  "/Users/rohansawhney/Desktop/developer/C++/model-repair/teapot.obj"};
Mesh mesh;
bool success = true;

void printInstructions()
{
    std::cerr << "space: toggle between meshes\n"
              << "↑/↓: move in/out\n"
              << "w/s: move up/down\n"
              << "a/d: move left/right\n"
              << "l : write to file\n"
              << "escape: exit program\n"
              << std::endl;
}

void init()
{
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glEnable(GL_DEPTH_TEST);
}

void draw()
{
    glLineWidth(2.0);
    glBegin(GL_LINES);
    for (EdgeIter e = mesh.edges.begin(); e != mesh.edges.end(); e++) {
        const Eigen::Vector3d &a(mesh.vertices[e->v0].position);
        const Eigen::Vector3d &b(mesh.vertices[e->v1].position);
        
        if (e->isBoundary) glColor4f(0.0, 0.6, 0.0, 0.5);
        else glColor4f(0.0, 0.0, 0.6, 0.5);
        
        glVertex3d(a.x(), a.y(), a.z());
        glVertex3d(b.x(), b.y(), b.z());
    }
    glEnd();
}

void display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    double aspect = (double)viewport[2] / (double)viewport[3];
    gluPerspective(fovy, aspect, clipNear, clipFar);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    gluLookAt(eyeX, eyeY, eyeZ, x, y, z, upX, upY, upZ);
    
    if (success) {
        draw();
    }

    glutSwapBuffers();
}

void keyboard(unsigned char key, int x0, int y0)
{
    switch (key) {
        case 27 :
            exit(0);
        case ' ':
            static size_t i = 0;
            i++;
            if (i == paths.size()) i = 0;
            mesh.read(paths[i]);
            break;
        case 'a':
            x -= 0.03;
            break;
        case 'd':
            x += 0.03;
            break;
        case 'w':
            y += 0.03;
            break;
        case 's':
            y -= 0.03;
            break;
        case 'l':
            std::string wPath = paths[i];
            mesh.write(wPath.insert(paths[i].find_last_of("."), "2"));
            break;
    }
    
    glutPostRedisplay();
}

void mouse(int x, int y)
{
    // Mouse point to angle conversion
    theta = (360.0 / gridY)*y*3.0;    // 3.0 rotations possible
   	phi = (360.0 / gridX)*x*3.0;
    
    // Restrict the angles within 0~360 deg (optional)
   	if (theta > 360) theta = fmod((double)theta, 360.0);
   	if (phi > 360) phi = fmod((double)phi, 360.0);
    
    // Spherical to Cartesian conversion.
    // Degrees to radians conversion factor 0.0174532
    eyeX = r * sin(theta*0.0174532) * sin(phi*0.0174532);
    eyeY = r * cos(theta*0.0174532);
   	eyeZ = r * sin(theta*0.0174532) * cos(phi*0.0174532);
    
    // Reduce theta slightly to obtain another point on the same longitude line on the sphere.
    GLfloat dt = 1.0;
   	GLfloat eyeXtemp = r * sin(theta*0.0174532-dt) * sin(phi*0.0174532);
   	GLfloat eyeYtemp = r * cos(theta*0.0174532-dt);
   	GLfloat eyeZtemp = r * sin(theta*0.0174532-dt) * cos(phi*0.0174532);
    
    // Connect these two points to obtain the camera's up vector.
   	upX = eyeXtemp - eyeX;
   	upY = eyeYtemp - eyeY;
   	upZ = eyeZtemp - eyeZ;
    
   	glutPostRedisplay();
}

void special(int i, int x0, int y0)
{
    switch (i) {
        case GLUT_KEY_UP:
            r -= 0.1;
            mouse(x0, y0);
            break;
        case GLUT_KEY_DOWN:
            r += 0.1;
            mouse(x0, y0);
            break;
    }
    
    glutPostRedisplay();
}

int main(int argc, char** argv) {
    
    printInstructions();
    success = mesh.read(paths[0]);
    
    glutInitWindowSize(gridX, gridY);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutInit(&argc, argv);
    glutCreateWindow("Model Repair");
    init();
    glutDisplayFunc(display);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(special);
    glutMotionFunc(mouse);
    glutMainLoop();
    
    return 0;
}
