#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <cmath>
#include <array>
#include <vector>
#include <string>
#include <cstdio>

#ifdef __APPLE__
  #include <GLUT/glut.h>
  #include <OpenGL/gl.h>
  #include <OpenGL/glu.h>
#else
  #include <GL/glut.h>
  #include <GL/gl.h>
  #include <GL/glu.h>
#endif

namespace py = pybind11;

int windowWidth = 800;
int windowHeight = 600;
double cameraAzimuth = 45.0;
double cameraElevation = 20.0;
double cameraDistance = 4.0;

py::dict pidInfo;

void initGL() {
    glClearColor(0.9f, 0.9f, 0.85f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_MULTISAMPLE);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    GLfloat lightPos[] = {4.0f, 4.0f, 4.0f, 1.0f};
    GLfloat ambientLight[] = {0.3f, 0.3f, 0.3f, 1.0f};
    GLfloat diffuseLight[] = {0.7f, 0.7f, 0.7f, 1.0f};
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
}

void drawText(float x, float y, const std::string &textString) {
    glRasterPos2f(x, y);
    for (char ch : textString)
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, ch);
}

void drawAxes() {
    float axisLength = 1.0f;
    glBegin(GL_LINES);
      glColor3f(1, 0, 0);
      glVertex3f(0, 0, 0);
      glVertex3f(axisLength, 0, 0);
      
      glColor3f(0, 1, 0);
      glVertex3f(0, 0, 0);
      glVertex3f(0, axisLength, 0);
      
      glColor3f(0, 0, 1);
      glVertex3f(0, 0, 0);
      glVertex3f(0, 0, axisLength);
    glEnd();
}

void drawReactionWheelLines() {
    int numSegments = 32;
    float radius = 0.2f;
    glColor3f(0, 0, 0);
    glBegin(GL_LINE_LOOP);
      for (int i = 0; i < numSegments; i++) {
          float theta = 2 * M_PI * i / numSegments;
          float x = radius * std::cos(theta);
          float y = radius * std::sin(theta);
          glVertex2f(x, y);
      }
    glEnd();
    for (int i = 0; i < 4; i++) {
        float angle = 2 * M_PI * i / 4;
        glBegin(GL_LINES);
          glVertex2f(0, 0);
          glVertex2f(radius * std::cos(angle), radius * std::sin(angle));
        glEnd();
    }
}

void drawErrorGraph() {
    float graphX = 10;
    float graphY = 10;
    float graphWidth = 200;
    float graphHeight = 100;
    
    glColor3f(0, 0, 0);
    glBegin(GL_LINE_LOOP);
      glVertex2f(graphX, graphY);
      glVertex2f(graphX + graphWidth, graphY);
      glVertex2f(graphX + graphWidth, graphY + graphHeight);
      glVertex2f(graphX, graphY + graphHeight);
    glEnd();
    
    py::list errorHistory = pidInfo["error_history"].cast<py::list>();
    if (errorHistory.size() > 1) {
        double tMin = 1e9, tMax = -1e9;
        std::vector<double> times, errors;
        for (auto item : errorHistory) {
            py::tuple tup = item.cast<py::tuple>();
            double t = tup[0].cast<double>();
            double e = tup[1].cast<double>();
            times.push_back(t);
            errors.push_back(e);
            if (t < tMin) tMin = t;
            if (t > tMax) tMax = t;
        }
        if (tMax == tMin) tMax = tMin + 1;
        double errMin = -1, errMax = 1;
        glColor3f(1, 0, 0);
        glBegin(GL_LINE_STRIP);
        for (size_t i = 0; i < times.size(); i++) {
            float xVal = graphX + float((times[i] - tMin) / (tMax - tMin)) * graphWidth;
            float yVal = graphY + float((errors[i] - errMin) / (errMax - errMin)) * graphHeight;
            glVertex2f(xVal, yVal);
        }
        glEnd();
    }
}

void display(py::object cubeSatObj) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    
    double az = cameraAzimuth * M_PI / 180.0;
    double el = cameraElevation * M_PI / 180.0;
    double camX = cameraDistance * std::cos(el) * std::sin(az);
    double camY = cameraDistance * std::sin(el);
    double camZ = cameraDistance * std::cos(el) * std::cos(az);
    gluLookAt(camX, camY, camZ, 0, 0, 0, 0, 1, 0);
    
    drawAxes();
    
    py::object orientationObj = cubeSatObj.attr("orientation");
    std::vector<double> orientation = orientationObj.cast<std::vector<double>>();
    std::array<double, 4> quat;
    for (int i = 0; i < 4; i++)
        quat[i] = orientation[i];
        
    std::array<float, 16> rotMat = {
       float(1 - 2 * (quat[2]*quat[2] + quat[3]*quat[3])),
       float(2 * (quat[1]*quat[2] - quat[3]*quat[0])),
       float(2 * (quat[1]*quat[3] + quat[2]*quat[0])),
       0.0f,
       
       float(2 * (quat[1]*quat[2] + quat[3]*quat[0])),
       float(1 - 2 * (quat[1]*quat[1] + quat[3]*quat[3])),
       float(2 * (quat[2]*quat[3] - quat[1]*quat[0])),
       0.0f,
       
       float(2 * (quat[1]*quat[3] - quat[2]*quat[0])),
       float(2 * (quat[2]*quat[3] + quat[1]*quat[0])),
       float(1 - 2 * (quat[1]*quat[1] + quat[2]*quat[2])),
       0.0f,
       
       0.0f, 0.0f, 0.0f, 1.0f
    };
    
    std::array<float, 16> rotMatTranspose;
    for (int i = 0; i < 4; i++)
      for (int j = 0; j < 4; j++)
         rotMatTranspose[i*4+j] = rotMat[j*4+i];

    glPushMatrix();
       glMultMatrixf(rotMatTranspose.data());
       py::object rwOffsetObj = cubeSatObj.attr("reactionWheelOffset");
       std::vector<double> rwOffset = rwOffsetObj.cast<std::vector<double>>();
       glPushMatrix();
          if (rwOffset.size() >= 3)
              glTranslatef(rwOffset[0], rwOffset[1], rwOffset[2]);
          py::object rwObj = cubeSatObj.attr("reactionWheel");
          double rwAngle = rwObj.attr("angle").cast<double>();
          glRotatef(rwAngle * 180.0 / M_PI, 0, 0, 1);
          drawReactionWheelLines();
       glPopMatrix();
    glPopMatrix();
    
    glDepthMask(GL_FALSE);
    glPushMatrix();
       glMultMatrixf(rotMatTranspose.data());
       glColor4f(0.2f, 0.7f, 0.3f, 0.3f);
       glutSolidCube(1.0);
       glColor4f(0.2f, 0.7f, 0.3f, 0.5f);
       glutWireCube(1.02);
    glPopMatrix();
    glDepthMask(GL_TRUE);

    float arrowLength = 1.5f;
    double curAngle = 2 * std::atan2(quat[3], quat[0]);
    glColor3f(1, 0, 0);
    glBegin(GL_LINES);
       glVertex3f(0, 0, 0);
       glVertex3f(arrowLength * std::cos(curAngle), arrowLength * std::sin(curAngle), 0);
    glEnd();
    double targetAngle = M_PI / 4;
    glColor3f(0, 0, 1);
    glBegin(GL_LINES);
       glVertex3f(0, 0, 0);
       glVertex3f(arrowLength * std::cos(targetAngle), arrowLength * std::sin(targetAngle), 0);
    glEnd();
    glColor3f(0, 1, 0);
    glBegin(GL_LINES);
       glVertex3f(arrowLength * std::cos(curAngle), arrowLength * std::sin(curAngle), 0);
       glVertex3f(arrowLength * std::cos(targetAngle), arrowLength * std::sin(targetAngle), 0);
    glEnd();
    
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
       glLoadIdentity();
       gluOrtho2D(0, windowWidth, 0, windowHeight);
       glMatrixMode(GL_MODELVIEW);
       glPushMatrix();
          glLoadIdentity();
          glColor3f(0, 0, 0);
          float xOffset = 10;
          float yOffset = windowHeight - 20;
          float lineSpacing = 15;
          std::vector<std::string> infoLines;
          char buffer[128];
          std::snprintf(buffer, sizeof(buffer), "PID Error: %.4f rad", atof(py::str(pidInfo["error"]).cast<std::string>().c_str()));
          infoLines.push_back(buffer);
          std::snprintf(buffer, sizeof(buffer), "P Term: %.4f", atof(py::str(pidInfo["p_term"]).cast<std::string>().c_str()));
          infoLines.push_back(buffer);
          std::snprintf(buffer, sizeof(buffer), "I Term: %.4f", atof(py::str(pidInfo["i_term"]).cast<std::string>().c_str()));
          infoLines.push_back(buffer);
          std::snprintf(buffer, sizeof(buffer), "D Term: %.4f", atof(py::str(pidInfo["d_term"]).cast<std::string>().c_str()));
          infoLines.push_back(buffer);
          std::snprintf(buffer, sizeof(buffer), "Motor Cmd: %.6f", atof(py::str(pidInfo["motor_command"]).cast<std::string>().c_str()));
          infoLines.push_back(buffer);
          std::snprintf(buffer, sizeof(buffer), "Wheel Speed: %.4f rad/s", atof(py::str(pidInfo["reaction_wheel_speed"]).cast<std::string>().c_str()));
          infoLines.push_back(buffer);
          std::snprintf(buffer, sizeof(buffer), "Sat Angle: %.4f rad", atof(py::str(pidInfo["sat_angle"]).cast<std::string>().c_str()));
          infoLines.push_back(buffer);
          std::snprintf(buffer, sizeof(buffer), "Sat ω: %.4f rad/s", atof(py::str(pidInfo["sat_angular_velocity"]).cast<std::string>().c_str()));
          infoLines.push_back(buffer);
          std::snprintf(buffer, sizeof(buffer), "Sat α: %.4f rad/s²", atof(py::str(pidInfo["sat_angular_acceleration"]).cast<std::string>().c_str()));
          infoLines.push_back(buffer);
          std::snprintf(buffer, sizeof(buffer), "RW Angle: %.4f rad", atof(py::str(pidInfo["rw_angle"]).cast<std::string>().c_str()));
          infoLines.push_back(buffer);
          std::snprintf(buffer, sizeof(buffer), "RW ω: %.4f rad/s", atof(py::str(pidInfo["rw_angular_velocity"]).cast<std::string>().c_str()));
          infoLines.push_back(buffer);
          std::snprintf(buffer, sizeof(buffer), "RW α: %.4f rad/s²", atof(py::str(pidInfo["rw_angular_acceleration"]).cast<std::string>().c_str()));
          infoLines.push_back(buffer);
          std::snprintf(buffer, sizeof(buffer), "Sim Time: %.2f s", atof(py::str(pidInfo["sim_time"]).cast<std::string>().c_str()));
          infoLines.push_back(buffer);
          std::snprintf(buffer, sizeof(buffer), "Real Time: %.2f s", atof(py::str(pidInfo["real_time"]).cast<std::string>().c_str()));
          infoLines.push_back(buffer);
          std::snprintf(buffer, sizeof(buffer), "Time Scale: %.2f", atof(py::str(pidInfo["time_scale"]).cast<std::string>().c_str()));
          infoLines.push_back(buffer);
          std::snprintf(buffer, sizeof(buffer), "Camera Azimuth: %.1f°", cameraAzimuth);
          infoLines.push_back(buffer);
          std::snprintf(buffer, sizeof(buffer), "Camera Elevation: %.1f°", cameraElevation);
          infoLines.push_back(buffer);
          for (const auto &line : infoLines) {
              drawText(xOffset, yOffset, line);
              yOffset -= lineSpacing;
          }
          drawErrorGraph();
       glPopMatrix();
       glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glutSwapBuffers();
}

void reshapeWindow(int w, int h) {
    windowWidth = w;
    windowHeight = h;
    if (h == 0)
        h = 1;
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45, (float)w/h, 0.1, 50.0);
    glMatrixMode(GL_MODELVIEW);
}

PYBIND11_MODULE(graphics, m) {
    m.doc() = "ADCS graphics simulation module (C++)";
    m.def("initGL", &initGL, "Initialize OpenGL");
    m.def("drawText", &drawText, "Render text on screen");
    m.def("drawAxes", &drawAxes, "Draw coordinate axes");
    m.def("drawReactionWheelLines", &drawReactionWheelLines, "Draw reaction wheel lines");
    m.def("drawErrorGraph", &drawErrorGraph, "Draw error graph");
    m.def("display", &display, "Render the simulation scene", py::arg("cubeSat"));
    m.def("reshapeWindow", &reshapeWindow, "Reshape the window", py::arg("w"), py::arg("h"));

    m.attr("windowWidth") = windowWidth;
    m.attr("windowHeight") = windowHeight;
    m.attr("cameraAzimuth") = cameraAzimuth;
    m.attr("cameraElevation") = cameraElevation;
    m.attr("cameraDistance") = cameraDistance;
    
    pidInfo = py::dict();
    pidInfo["error"] = 0.0;
    pidInfo["p_term"] = 0.0;
    pidInfo["i_term"] = 0.0;
    pidInfo["d_term"] = 0.0;
    pidInfo["motor_command"] = 0.0;
    pidInfo["reaction_wheel_speed"] = 0.0;
    pidInfo["sim_time"] = 0.0;
    pidInfo["real_time"] = 0.0;
    pidInfo["time_scale"] = 0.0;
    pidInfo["sat_angle"] = 0.0;
    pidInfo["sat_angular_velocity"] = 0.0;
    pidInfo["sat_angular_acceleration"] = 0.0;
    pidInfo["rw_angle"] = 0.0;
    pidInfo["rw_angular_velocity"] = 0.0;
    pidInfo["rw_angular_acceleration"] = 0.0;
    pidInfo["error_history"] = py::list();
    m.attr("pidInfo") = pidInfo;
}