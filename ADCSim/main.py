import sys
import time
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import graphics      # C++ module
from physics import CubeSat, getTrueAngle
from control import controlSystemPID

# timestep (virtual seconds per simulation tick)
dt = 0.01

cubeSat = CubeSat()
simTime = 0.0
realTimeStart = time.time()

prevSatOmega = 0.0
prevRwOmega = 0.0

def timerCallback(value):
    global simTime, cubeSat, prevSatOmega, prevRwOmega
    motorCommand, error, pTerm, iTerm, dTerm = controlSystemPID(cubeSat, dt)
    
    graphics.pidInfo["error"] = error
    graphics.pidInfo["p_term"] = pTerm
    graphics.pidInfo["i_term"] = iTerm
    graphics.pidInfo["d_term"] = dTerm
    graphics.pidInfo["motor_command"] = motorCommand
    graphics.pidInfo["reaction_wheel_speed"] = cubeSat.reactionWheel.angularSpeed

    cubeSat.update(motorCommand, dt)

    simTime += dt
    realTimeElapsed = time.time() - realTimeStart
    timeScale = simTime / realTimeElapsed if realTimeElapsed > 0 else 0

    graphics.pidInfo["sim_time"] = simTime
    graphics.pidInfo["real_time"] = realTimeElapsed
    graphics.pidInfo["time_scale"] = timeScale

    satAngle = getTrueAngle(cubeSat)
    graphics.pidInfo["sat_angle"] = satAngle

    currSatOmega = cubeSat.angularVelocity[2]
    graphics.pidInfo["sat_angular_velocity"] = currSatOmega
    graphics.pidInfo["sat_angular_acceleration"] = (currSatOmega - prevSatOmega) / dt
    prevSatOmega = currSatOmega

    currRwOmega = cubeSat.reactionWheel.angularSpeed
    graphics.pidInfo["rw_angular_velocity"] = currRwOmega
    graphics.pidInfo["rw_angular_acceleration"] = (currRwOmega - prevRwOmega) / dt
    prevRwOmega = currRwOmega

    graphics.pidInfo["rw_angle"] = cubeSat.reactionWheel.angle

    graphics.pidInfo["error_history"].append((simTime, error))
    if len(graphics.pidInfo["error_history"]) > 1000:
        graphics.pidInfo["error_history"].pop(0)

    glutPostRedisplay()
    glutTimerFunc(int(dt * 1000), timerCallback, 0)

# space resets, esc exits (not really its kinda broken just alt+f4)
def keyboardCallback(key, x, y):
    global cubeSat, simTime, realTimeStart
    if key == b' ':
        cubeSat = CubeSat()
        simTime = 0.0
        realTimeStart = time.time()
    elif key == b'\x1b':
        sys.exit()

# arrow keys orbit the camera
# TODO: FIX, Broken since moving to cpp
def specialKeys(key, x, y):
    from OpenGL.GLUT import GLUT_KEY_LEFT, GLUT_KEY_RIGHT, GLUT_KEY_UP, GLUT_KEY_DOWN
    if key == GLUT_KEY_LEFT:
        graphics.cameraAzimuth -= 5
    elif key == GLUT_KEY_RIGHT:
        graphics.cameraAzimuth += 5
    elif key == GLUT_KEY_UP:
        graphics.cameraElevation = min(graphics.cameraElevation + 5, 89)
    elif key == GLUT_KEY_DOWN:
        graphics.cameraElevation = max(graphics.cameraElevation - 5, -89)
    glutPostRedisplay()

def main():
    glutInit(sys.argv)
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_MULTISAMPLE)
    glutInitWindowSize(graphics.windowWidth, graphics.windowHeight)
    glutCreateWindow(b"Control System Simulation")
    glEnable(GL_DEPTH_TEST)
    graphics.initGL()
    glutDisplayFunc(lambda: graphics.display(cubeSat))
    glutReshapeFunc(graphics.reshapeWindow)
    glutKeyboardFunc(keyboardCallback)
    glutSpecialFunc(specialKeys)
    glutTimerFunc(int(dt * 1000), timerCallback, 0)
    glutMainLoop()

if __name__ == '__main__':
    main()