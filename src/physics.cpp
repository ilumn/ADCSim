#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <array>
#include <cmath>
#include <random>

namespace py = pybind11;

// placeholders for moment of inertia, works well enough for the simulation but later I hope to make it physically accurate using provided cad files for the sat and wheel
static const double iSat = 0.01;
static const double iWheel = 0.001;
static const double piVal = 3.14159265358979323846;

using Quaternion = std::array<double, 4>;
using Vec3 = std::array<double, 3>;

// Returns the Hamilton product of two quaternions
inline Quaternion quaternionMultiply(const Quaternion &q, const Quaternion &r) {
    return { q[0]*r[0] - q[1]*r[1] - q[2]*r[2] - q[3]*r[3],
             q[0]*r[1] + q[1]*r[0] + q[2]*r[3] - q[3]*r[2],
             q[0]*r[2] - q[1]*r[3] + q[2]*r[0] + q[3]*r[1],
             q[0]*r[3] + q[1]*r[2] - q[2]*r[1] + q[3]*r[0] };
}

// Returns the conjugate of a quaternion
inline Quaternion quaternionConjugate(const Quaternion &q) {
    return { q[0], -q[1], -q[2], -q[3] };
}

// Converts a quaternion to a 4x4 rotation matrix
std::array<float, 16> quaternionToMatrix(const Quaternion &q) {
    double w = q[0], x = q[1], y = q[2], z = q[3];
    return {
            float(1 - 2*(y*y + z*z)),      float(2*(x*y - z*w)),           float(2*(x*z + y*w)),           0.0f,
            float(2*(x*y + z*w)),          float(1 - 2*(x*x + z*z)),       float(2*(y*z - x*w)),           0.0f,
            float(2*(x*z - y*w)),          float(2*(y*z + x*w)),           float(1 - 2*(x*x + y*y)),       0.0f,
            0.0f,                          0.0f,                           0.0f,                           1.0f
        };
}

double getTrueAngle(const py::object &cubeSatObj);

// Net torque (actuator minus damping) gives angular acceleration
class ReactionWheel {
public:
    double I;
    double motorConstant;
    double damping;
    double angularSpeed;
    double angle;

    ReactionWheel(double I_ = iWheel, double motorConstant_ = 0.02, double damping_ = 0.0005)
      : I(I_), motorConstant(motorConstant_), damping(damping_), angularSpeed(0.0), angle(0.0) {}

    // gets torque from a reaction wheel input command, the idea is to simulate the wheel spin with a given command to it's motor
    double update(double command, double dt) {
        double torque = motorConstant * command - damping * angularSpeed;
        angularSpeed += (torque / I) * dt;
        angle += angularSpeed * dt;
        angle = fmod(angle, 2 * piVal);
        if(angle < 0) angle += 2 * piVal;
        return torque;
    }
};

// Reaction wheel torque produces an equal and opposite body torque
// The orientation is updated using the exponential map for exact integration.
// q = [cos(|ω| dt/2), (ω/|ω|) sin(|ω| dt/2)]
class CubeSat {
public:
    double I;
    Quaternion orientation;
    Vec3 angularVelocity;
    ReactionWheel reactionWheel;
    Vec3 reactionWheelOffset;

    CubeSat(double I_ = iSat)
      : I(I_), orientation({1.0, 0.0, 0.0, 0.0}),
        angularVelocity({0.0, 0.0, 0.0}),
        reactionWheel(), reactionWheelOffset({0.0, 0.0, 0.0}) {}

    // Updates the CubeSat state for the given motor command over dt
    void update(double motorCommand, double dt) {
        double torque = reactionWheel.update(motorCommand, dt);
        Vec3 reactionTorque = {0.0, 0.0, -torque};
        for (int i = 0; i < 3; i++)
            angularVelocity[i] += (reactionTorque[i] / I) * dt;
        integrateOrientation(dt);
        normalize(orientation);
    }

private:
    // Integrates orientation using the exponential map.
    // q(t+dt) = exp((dt/2)*ω̂) ⊗ q(t)
    void integrateOrientation(double dt) {
        double wx = angularVelocity[0], wy = angularVelocity[1], wz = angularVelocity[2];
        double omegaNorm = std::sqrt(wx*wx + wy*wy + wz*wz);
        Quaternion deltaQ;
        if (omegaNorm < 1e-10) {
            deltaQ = {1.0, 0.0, 0.0, 0.0};
        } else {
            double theta = omegaNorm * dt;
            double halfTheta = theta / 2.0;
            double sinHalfTheta = std::sin(halfTheta);
            double cosHalfTheta = std::cos(halfTheta);
            deltaQ = { cosHalfTheta,
                       sinHalfTheta * (wx / omegaNorm),
                       sinHalfTheta * (wy / omegaNorm),
                       sinHalfTheta * (wz / omegaNorm) };
        }
        orientation = quaternionMultiply(deltaQ, orientation);
    }

    void normalize(Quaternion &q) {
        double norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
        for (auto &v : q)
            v /= norm;
    }
};

// Returns the true rotation angle (z-axis) from the CubeSat quaternion.
// TODO: return vec3 with x,y,z rotation angles
double getTrueAngle(const CubeSat &cubeSat) {
    return 2 * std::atan2(cubeSat.orientation[3], cubeSat.orientation[0]);
}

double getTrueAngle(const py::object &cubeSatObj) {
    CubeSat *cs = cubeSatObj.cast<CubeSat*>();
    return getTrueAngle(*cs);
}

// Virtual sensor returns noisy true Z-axis angle
// the simulation is too easy if there is no noise, it can just go exactly to the target angle without needing to correct.
double getVirtualSensorAngle(const CubeSat &cubeSat, double noiseStd = 0.005) {
    double trueAngle = getTrueAngle(cubeSat);
    static std::default_random_engine generator(std::random_device{}());
    std::normal_distribution<double> distribution(0.0, noiseStd);
    return trueAngle + distribution(generator);
}

// Virtual gyro returns satellite angular velocity with noise.
Vec3 getVirtualGyro(const CubeSat &cubeSat, double noiseStd = 0.001) {
    Vec3 noisy;
    static std::default_random_engine generator(std::random_device{}());
    std::normal_distribution<double> distribution(0.0, noiseStd);
    for (int i = 0; i < 3; i++)
        noisy[i] = cubeSat.angularVelocity[i] + distribution(generator);
    return noisy;
}

// Virtual accelerometer computes acceleration from gravity
Vec3 getVirtualAccelerometer(const CubeSat &cubeSat, double noiseStd = 0.01) {
    Vec3 g = {0.0, -9.81, 0.0};
    std::array<float, 16> m = quaternionToMatrix(cubeSat.orientation);
    Vec3 aBody = {0.0, 0.0, 0.0};
    for (int j = 0; j < 3; j++) {
        for (int i = 0; i < 3; i++) {
            aBody[j] += m[i*4 + j] * g[i];
        }
    }
    static std::default_random_engine generator(std::random_device{}());
    std::normal_distribution<double> distribution(0.0, noiseStd);
    for (int i = 0; i < 3; i++)
        aBody[i] += distribution(generator);
    return aBody;
}

PYBIND11_MODULE(physics, m) {
    m.doc() = "CubeSat physics simulation module (C++)";

    m.def("quaternionMultiply", &quaternionMultiply, "Multiply two quaternions");
    m.def("quaternionConjugate", &quaternionConjugate, "Conjugate of a quaternion");
    m.def("quaternionToMatrix", &quaternionToMatrix, "Convert quaternion to rotation matrix");
    m.def("getTrueAngle", (double(*)(const CubeSat&)) &getTrueAngle, "Compute CubeSat true angle");

    m.def("getVirtualSensorAngle", &getVirtualSensorAngle, "Simulate a virtual sensor reading",
          py::arg("cubeSat"), py::arg("noiseStd") = 0.005);
    m.def("getVirtualGyro", &getVirtualGyro, "Simulate a virtual gyro reading",
          py::arg("cubeSat"), py::arg("noiseStd") = 0.001);
    m.def("getVirtualAccelerometer", &getVirtualAccelerometer, "Simulate a virtual accelerometer reading",
          py::arg("cubeSat"), py::arg("noiseStd") = 0.01);

    py::class_<ReactionWheel>(m, "ReactionWheel")
        .def(py::init<double, double, double>(),
             py::arg("I") = iWheel, py::arg("motorConstant") = 0.02, py::arg("damping") = 0.0005)
        .def("update", &ReactionWheel::update)
        .def_readwrite("angularSpeed", &ReactionWheel::angularSpeed)
        .def_readwrite("angle", &ReactionWheel::angle);

    py::class_<CubeSat>(m, "CubeSat")
        .def(py::init<double>(), py::arg("I") = iSat)
        .def("update", &CubeSat::update)
        .def_readwrite("orientation", &CubeSat::orientation)
        .def_readwrite("angularVelocity", &CubeSat::angularVelocity)
        .def_readwrite("reactionWheel", &CubeSat::reactionWheel)
        .def_readwrite("reactionWheelOffset", &CubeSat::reactionWheelOffset);
}