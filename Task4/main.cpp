#include <iostream>
#include <opencv2/opencv.hpp>
#include <GL/glut.h> // GLUT, include glu.h and gl.h

cv::VideoCapture *cap;

void display() {
    using namespace cv;
    using namespace std;

    Mat frame;
    bool bSuccess = cap->read(frame); // read a new frame from video
    if (!bSuccess) {
        cout << "Cannot read the frame from video file" << endl;
        exit(0);
    }

    GLuint texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture); // 2d texture (x and y size)
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR); // scale linearly when image bigger than texture
    glTexImage2D(GL_TEXTURE_2D, 0, 3, frame.cols, frame.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, frame.data);
    glBindTexture(GL_TEXTURE_2D, texture); // choose the texture to use

    // Draw a Red 1x1 Square centered at origin
    glBegin(GL_QUADS); // Each set of 4 vertices form a quad
    glColor3f(1.0f, 0.0f, 0.0f); // Red
    float x = 0.1;
    float y = 0.1;
    glTexCoord2f(0.0f, 0.0f); glVertex2f(-x, -y);
    glTexCoord2f(0.0f, 1.0f); glVertex2f( x, -y);
    glTexCoord2f(1.0f, 1.0f); glVertex2f( x, y);
    glTexCoord2f(1.0f, 0.0f); glVertex2f(-x, y);
    glEnd();
    glFlush(); // Render now
}

void idle() {
    glutPostRedisplay(); // Post a re-paint request to activate display()
}

// Консольное приложение с glut
int main(int argc, char** argv) {
    using namespace cv;
    using namespace std;

    cap = new VideoCapture("video.mov"); // open the video file for reading
    if ( !cap->isOpened() ) return -1;
    double fps = cap->get(CV_CAP_PROP_FPS); //get the frames per seconds of the video
    cout << "Frame per seconds : " << fps << endl;

    glutInit(&argc, argv); // Инициализация GLUT
    glutCreateWindow("OpenGL Setup Test"); // Создать окно
    glutInitWindowSize(320, 320); // Размер окна
    glutInitWindowPosition(50, 50); // Позиция окна относитель лев.верхн. угла
    glutDisplayFunc(display); // Коллбак для перерисовки окна
    // Для выполнения анимации возможно зарегистрировать коллбак idle()
    // при помощи glutIdleFunc. Он будет вызываться, когда система свободна.
    glutIdleFunc(idle);
    glutInitDisplayMode(GLUT_DOUBLE); // Set double buffered mode
    glutMainLoop(); // Цикл обработки событий

    return 0;
}