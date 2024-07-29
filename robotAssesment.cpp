#include <Eigen/Dense>
#include <cmath>
#include <iostream>

using namespace std;
using namespace Eigen;

/**
 * Computes the Jacobian matrix for a given set of parameters.
 *
 * @param r Radius value.
 * @param L Length value.
 * @param d Distance value.
 * @return Computed Jacobian matrix.
 */
MatrixXd getJacobianMatrix(double r, double L, double d) {
    MatrixXd J(4, 3);

    J << 1, 1, -(L + d),
         -1, 1, (L + d),
         -1, 1, (L + d),
         1, 1, -(L + d);

    return J;
}

/**
 * Computes the rotation matrix from the local coordinate 𝜗𝑅{𝐺𝑥𝑅𝑦𝑅} (mounted on the robot) to respect the
 * global coordinate 𝜗f
 *
 * @param phi Angle of rotation of the encoder.
 * @return Computed transformation matrix.
 */
MatrixXd getTransMatrix(double phi) {
    MatrixXd Q(3, 3);

    Q << cos(phi), -sin(phi), 0,
         sin(phi),  cos(phi), 0,
                0,         0, 1;

    return Q;
}

/**
 * Computes the vector of generalized coordinates in global coordinate 𝜗𝑓.
 *
 * @param J Jacobian matrix.
 * @param Q Transformation matrix.
 * @param w Angular velocity vector.
 * @return Computed rotation vector.
 */
VectorXd getRotationMatrix(MatrixXd J, MatrixXd Q, double w) {
    // 𝑞̇ = 𝑄(𝜙)(𝐽𝑇𝐽)−1𝐽𝑇𝜔
    VectorXd q(3);

    // Calcular J^T J y su inversa
    MatrixXd JJ_inv = (J.transpose() * J).inverse();

    // Calcular J^T y w
    Vector3d J_transpose_w = J.transpose() * w;  //

    // Calcular q
    return (Q * JJ_inv * J_transpose_w);
}

int main_robot() {
    // Definición de variables
    float r = 0;      // radio de la llanta
    float L = 0;      // distancia entre las dos llantas
    float d = 0;      // distancia entre llantas delanteras y traseras
    float w = 0;      // velocidad del control
    float phi = 0.0;  // Configuración angular de las articulaciones : 𝜙 : encoder

    float vx = 0;
    float vy = 0;

    // Calcular vectores y matrices
    MatrixXd J = getJacobianMatrix(r, L, d);
    MatrixXd Q = getTransMatrix(phi);
    VectorXd q = getRotationMatrix(J, Q, w);

    VectorXd velR(3);
    velR << vx, vy, phi;

    // The relationship between the wheels angular velocities and the robot linear and angular velocities is determined
    // by:
    // 𝜔 = JQ𝑇(𝜙)𝑞̇
    VectorXd angularVelocity(4);
    angularVelocity = 1 / r * J * velR * Q.transpose() * q;
}