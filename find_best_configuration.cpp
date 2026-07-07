#include <iostream>
#include <array> 
#include <vector>
#include <chrono>
#include "Eigen/Dense"
#include <iomanip>

#include "geofik.h"

// compile with: g++ find_best_configuration.cpp geofik.cpp -I/usr/include/eigen3 -O3 -o find_best_configuration && ./find_best_configuration

void print_results(const array<array<double, 7>, 8>& sols, const bool swivel = false, const double theta = 0.0);
void print_results_J(const array<array<array<double, 6>, 7>, 8>& Jsols, const array<array<double, 7>, 8>& qsols, const bool joint_angles, const bool swivel = false, const double theta = 0.0);
array<double,9> return_ROE_from_O_T_EE(const array<double, 16>& O_T_EE);
array<double,3> return_r_from_O_T_EE(const array<double, 16>& O_T_EE);
array<double,4> return_quat_from_O_T_EE(const array<double, 16>& O_T_EE);
array<double,16> return_O_T_F_from_O_T_EE(const array<double, 16>& O_T_EE, const array<double, 16>& F_T_EE);
Eigen::Matrix<double, 6, 7> array_to_eigen(const array<array<double, 6>, 7>& J);

int main() {

    unsigned int nsols = 0; 
    array<double, 16> O_T_EE, O_T_F, F_T_EE; // Transformation matrix copied from echo_robot_state
    array<double, 9> ROE; // rotation matrix of frame E with respect to frame O
    array<double, 3> r; // position vector of frame E with respect to frame O
    double q7, q6, q4, theta; // free variables
    bool joint_angles;
    array<array<double, 7>, 8> qsols;
    array<array<array<double, 6>, 7>, 8> Jsols; 
    array<Eigen::Matrix<double, 6, 7>, 8> Jsols_eigen; 
    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    // -----------------------------------------------
    // ------    IK calculation       ----------------
    // -----------------------------------------------

    cout << endl << "=======================================================" << endl;
    cout << "franka_J_ik_swivel()" << endl;
    cout << "=======================================================" << endl;

    // Set target position/orientation
    O_T_EE = {-0.28893,-0.774153,-0.563211,0,-0.956799,0.253474,0.142433,0,0.0324947,0.580032,-0.813945,0,0.408563,-0.0423964,0.37532,1};
    F_T_EE = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0.02,1};
    O_T_F = return_O_T_F_from_O_T_EE(O_T_EE, F_T_EE);
    ROE = return_ROE_from_O_T_EE(O_T_EE);
    r = return_r_from_O_T_EE(O_T_EE);   

    // Test for loop to find best orientation for push
    Eigen::Matrix<double, 6, 1> force_desired;
    Eigen::Matrix<double, 6, 7> Jsol_1;
    force_desired.head(3) << 0,0,1;
    double angle_min = -PI/2;
    double angle_max =  PI/2;
    int n_angles = 20;
    for (int i=0; i<n_angles; i++) {
        // Set swivel angle
        const double t = static_cast<double>(i) / static_cast<double>(n_angles);
        theta = angle_min + t * (angle_max - angle_min);
        joint_angles = true;
        nsols = franka_J_ik_swivel(r, ROE, theta, Jsols, qsols, joint_angles, 'E');
        // print_results_J(Jsols, qsols, joint_angles);
        Jsol_1 = array_to_eigen(Jsols[0]);
        std::cout << "Necessary torque for swivel angle " << theta << ": \t" << (Jsol_1.transpose() * force_desired).norm() << " Nm" << std::endl;
    }
    
    return 0;
}



// Functions to return rotation matrix and position from T
array<double, 9> return_ROE_from_O_T_EE(const array<double, 16>& O_T_EE) {
    array<double, 9> ROE = {
        O_T_EE[0], O_T_EE[4], O_T_EE[8],
        O_T_EE[1], O_T_EE[5], O_T_EE[9],
        O_T_EE[2], O_T_EE[6], O_T_EE[10]};
    return ROE;
}
array<double, 3> return_r_from_O_T_EE(const array<double, 16>& O_T_EE) {
    array<double, 3> r = {O_T_EE[12], O_T_EE[13], O_T_EE[14]};
    return r;
}
// O_T_EE is column-major (Franka convention), so it maps directly onto an Eigen matrix
array<double, 4> return_quat_from_O_T_EE(const array<double, 16>& O_T_EE) {
    Eigen::Map<const Eigen::Matrix4d> T(O_T_EE.data());
    Eigen::Quaterniond quat(T.block<3, 3>(0, 0));
    return {quat.x(), quat.y(), quat.z(), quat.w()};
}
// Computes O_T_F (base -> flange), given O_T_EE and F_T_EE. All matrices are column-major (Franka convention).
array<double, 16> return_O_T_F_from_O_T_EE(const array<double, 16>& O_T_EE, const array<double, 16>& F_T_EE) {
    Eigen::Map<const Eigen::Matrix4d> T_O_EE(O_T_EE.data());
    Eigen::Map<const Eigen::Matrix4d> T_F_EE(F_T_EE.data());
    Eigen::Matrix4d T_O_F = T_O_EE * T_F_EE.inverse();
    array<double, 16> O_T_F;
    Eigen::Map<Eigen::Matrix4d>(O_T_F.data()) = T_O_F;
    return O_T_F;
}
// Converts a Jacobian stored as 7 columns of 6 doubles (as returned by franka_J_ik_swivel) into an Eigen matrix
Eigen::Matrix<double, 6, 7> array_to_eigen(const array<array<double, 6>, 7>& J) {
    return Eigen::Map<const Eigen::Matrix<double, 6, 7>>(J[0].data());
}

// functions to print results
void print_results(const array<array<double, 7>, 8>& sols, const bool swivel, const double theta) {
    cout << "SOLUTIONS ...................." << endl;
    Eigen::Matrix4d T;
    double theta_res;
    bool sol_in_lims;
    for (int i = 0; i < sols.size(); i++) {
        sol_in_lims = true;
        cout << endl << "solution " << i + 1 << endl;
        for (int j = 0; j < 7; j++) {
            cout << "q_" << j + 1 << " = " << sols[i][j] * 180 / PI << endl;
            if (isnan(sols[i][j]))
                sol_in_lims = false;
        }
        // only print retrieved T if all joints are within limits:
        if (sol_in_lims) {
            T = franka_fk(sols[i]);
            cout << "Forward kinematics for this solution:" << endl;
            cout << T;
            cout << endl;
            if (swivel) {
                theta_res = franka_swivel(sols[i]);
                cout << "theta = " << theta_res << "rad or " << theta_res * 180 / PI << "degs" << endl;
                cout << "error = " << theta - theta_res << "degs" <<endl;
            }
        }
    }
}

void print_results_J(const array<array<array<double, 6>, 7>, 8>& Jsols, const array<array<double, 7>, 8>& qsols, const bool joint_angles, const bool swivel, const double theta) {
    cout << "SOLUTIONS ...................." << endl;
    double theta_res;
    std::cout << std::fixed << std::setprecision(5);
    bool sol_in_lims;
    Eigen::Matrix4d T;
    for (int i = 0; i < 8; i++) {
        cout << endl << "solution " << i + 1 << endl;
        cout << "Jacobian:" << endl;
        for (int j = 0; j < 7; j++) {
            for (int k = 0; k < 6; k++) {
                cout << std::setw(10) << Jsols[i][j][k];
            }
            cout << endl;
        }
        if (joint_angles) {
            sol_in_lims = true;
            cout << "joint angles" << endl;
            for (int j = 0; j < 7; j++) {
                cout << "q_" << j + 1 << " = " << qsols[i][j] << endl;
                if (isnan(qsols[i][j]))
                    sol_in_lims = false;
            }
            if (sol_in_lims) {
                T = franka_fk(qsols[i]);
                cout << "Forward kinematics for this solution:" << endl;
                cout << T;
                cout << endl;
                if (swivel) {
                    theta_res = franka_swivel(qsols[i]);
                    cout << "theta = " << theta_res << "rad or " << theta_res * 180 / PI << "degs" << endl;
                    cout << "error = " << theta - theta_res << endl;
                }
            }
        }
        cout << endl;
    }
}