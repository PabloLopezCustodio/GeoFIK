#include <iostream>
#include <array> 
#include <vector>
#include <chrono>
#include "Eigen/Dense"
#include <iomanip>
using namespace std;
using namespace std::chrono;

#include "geofik.h"

// compile with: g++ example_geofik.cpp geofik.cpp -O3 -o example_geofik.exe

void print_results(const array<array<double, 7>, 8>& sols, const bool swivel = false, const double theta = 0.0);
void print_results_J(const array<array<array<double, 6>, 7>, 8>& Jsols, const array<array<double, 7>, 8>& qsols, const bool joint_angles, const bool swivel = false, const double theta = 0.0);
array<double,9> return_ROE_from_O_T_EE(const array<double, 16>& O_T_EE);
array<double,3> return_r_from_O_T_EE(const array<double, 16>& O_T_EE);
array<double,4> return_quat_from_O_T_EE(const array<double, 16>& O_T_EE);
array<double,16> return_O_T_F_from_O_T_EE(const array<double, 16>& O_T_EE, const array<double, 16>& F_T_EE);

int main() {

    unsigned int nsols = 0; 
    array<double, 16> O_T_EE, O_T_F, F_T_EE; // Transformation matrix copied from echo_robot_state
    array<double, 9> ROE; // rotation matrix of frame E with respect to frame O
    array<double, 3> r; // position vector of frame E with respect to frame O
    double q7, q6, q4, theta; // free variables
    bool joint_angles;
    array<array<double, 7>, 8> qsols;
    array<array<array<double, 6>, 7>, 8> Jsols; 
    high_resolution_clock::time_point start = high_resolution_clock::now();
    high_resolution_clock::time_point end = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(end - start);

    // TEST franka_ik_q7()
/*
    cout << endl << "=======================================================" << endl;
    cout << "franka_ik_q7()" << endl;
    cout << "=======================================================" << endl;
    ROE = {0.6688331000000003,0.3170534383478098,0.6724130000000006,-0.6398146000000005,-0.2150772409286401,0.7378204999999999,0.3785492999999998,-0.9236984268883508,0.05900459999999996};
    r = {0.61674948,0.32278029,0.56790512};
    q7 = -0.37218362471412003 ;
    start = high_resolution_clock::now();
    nsols = franka_ik_q7(r, ROE, q7, qsols);
    end = high_resolution_clock::now();
    duration = duration_cast<microseconds>(end - start);
    print_results(qsols);
    cout << "number of solutions found:" << nsols;
    cout << endl << "duration (only q):" << duration.count() << endl;
    // second run
    start = high_resolution_clock::now();
    nsols = franka_ik_q7(r, ROE, q7, qsols);
    end = high_resolution_clock::now();
    duration = duration_cast<microseconds>(end - start);
    cout << endl << "duration in second run:" << duration.count() << endl;
*/

    // TEST franka_ik_q6()
    /*
    cout << endl << "=======================================================" << endl;
    cout << "franka_ik_q6()" << endl;
    cout << "=======================================================" << endl;
    //test parallel case:
    ROE = {-0.6575819999999999,0.22565347159914026,0.7187951000000004,0.5218621000000001,-0.5516899774048302,0.6506136000000007,0.5433652999999999,0.8029437126709107,0.24502140000000006};
    r = {0.65543571,0.5899264,0.59988412};
    q6 = 3.141592653589793 ;
    //test non-parallel case:
    //ROE = { -0.1192774000000002,0.8774473359489797,0.4646064,0.47138319999999956,0.4618910375721001,-0.7513018999999999,-0.8738254000000001,0.12939431432541965,-0.4687071000000001 };
    //r = { 0.43749227,-0.73488144,0.4951873 };
    //q6 = 2.6179938779914944;
    start = high_resolution_clock::now();
    nsols = franka_ik_q6(r, ROE, q6, qsols);
    end = high_resolution_clock::now();
    duration = duration_cast<microseconds>(end - start);
    print_results(qsols);
    cout << "number of solutions found:" << nsols;
    cout << endl << "duration:" << duration.count() << endl;
    // second run
    start = high_resolution_clock::now();
    nsols = franka_ik_q6(r, ROE, q6, qsols);
    end = high_resolution_clock::now();
    duration = duration_cast<microseconds>(end - start);
    cout << endl << "duration in second run:" << duration.count() << endl;
    */

    // TEST franka_ik_q4()
    /*
    cout << endl << "=======================================================" << endl;
    cout << "franka_ik_q4()" << endl;
    cout << "=======================================================" << endl;
    ROE = { 0.3522750000000002,0.8334980991082203,0.4256560000000001,-0.6332361000000003,-0.12262271641620001,0.764183,0.6891402000000002,-0.5387433117066003,0.48460420000000004 };
    r = { -0.26617644,0.31984145,0.76295964 };
    q4 = -2.3401949461109055;
    start = high_resolution_clock::now();
    nsols = franka_ik_q4(r, ROE, q4, qsols);
    end = high_resolution_clock::now();
    duration = duration_cast<microseconds>(end - start);
    print_results(qsols);
    cout << "number of solutions found:" << nsols;
    cout << endl << "duration:" << duration.count() << endl;
    //second run
    start = high_resolution_clock::now();
    nsols = franka_ik_q4(r, ROE, q4, qsols);
    end = high_resolution_clock::now();
    duration = duration_cast<microseconds>(end - start);
    cout << endl << "duration in second run:" << duration.count() << endl;
    */

    // TEST franka_ik_swivel()
    /*
    cout << endl << "=======================================================" << endl;
    cout << "franka_ik_swivel()" << endl;
    cout << "=======================================================" << endl;
    ROE = { -0.5878745000000002,-0.7990239702980599,-0.12635000000000007,-0.16720689999999983,0.2728366411870001,-0.9474186000000001,0.7914831,-0.5358366439507003,-0.293996 };
    r = { -0.15116089,-0.397596,0.47386124 };
    theta = -0.3490658503988659;
    start = high_resolution_clock::now();
    nsols = franka_ik_swivel(r, ROE, theta, qsols);
    end = high_resolution_clock::now();
    print_results(qsols, true, theta);
    duration = duration_cast<microseconds>(end - start);
    cout << "Number of solutions found: " << nsols;
    cout << endl << "duration:" << duration.count() << endl;
    */

    // TEST franka_J_ik_q7()
    /*
    cout << endl << "=======================================================" << endl;
    cout << "franka_J_ik_q7()" << endl;
    cout << "=======================================================" << endl;
    ROE = {0.6688331000000003,0.3170534383478098,0.6724130000000006,-0.6398146000000005,-0.2150772409286401,0.7378204999999999,0.3785492999999998,-0.9236984268883508,0.05900459999999996};
    r = {0.61674948,0.32278029,0.56790512};
    q7 = -0.37218362471412003 ;
    ROE = { -0.189536, 0.0420467, -0.980973,
             0.404078, -0.907217, -0.116958,
            -0.894873, -0.418557, 0.15496 };
    r = { 0.23189, -0.0815989, 0.607269 };
    q7 = 0.771925;
    joint_angles = true;
    start = high_resolution_clock::now();
    nsols = franka_J_ik_q7(r, ROE, q7, Jsols, qsols, joint_angles);
    end = high_resolution_clock::now();
    duration = duration_cast<microseconds>(end - start);
    print_results_J(Jsols, qsols, joint_angles);
    cout << "number of solutions found: " << nsols;
    cout << endl<<"duration:" << duration.count() << endl;
    */

    // TEST franka_J_ik_q6()
    /*
    cout << endl << "=======================================================" << endl;
    cout << "franka_J_ik_q6()" << endl;
    cout << "=======================================================" << endl;
    //test parallel case:
    //ROE = { -0.6575819999999999,0.22565347159914026,0.7187951000000004,0.5218621000000001,-0.5516899774048302,0.6506136000000007,0.5433652999999999,0.8029437126709107,0.24502140000000006 };
    //r = { 0.65543571,0.5899264,0.59988412 };
    //q6 = 3.141592653589793;
    //test non-parallel case:
    ROE = {-0.1192774000000002,0.8774473359489797,0.4646064,0.47138319999999956,0.4618910375721001,-0.7513018999999999,-0.8738254000000001,0.12939431432541965,-0.4687071000000001};
    r = {0.43749227,-0.73488144,0.4951873};
    q6 = 2.6179938779914944 ;
    joint_angles = true;
    start = high_resolution_clock::now();
    nsols = franka_J_ik_q6(r, ROE, q6, Jsols, qsols, joint_angles, '8');
    end = high_resolution_clock::now();
    duration = duration_cast<microseconds>(end - start);
    print_results_J(Jsols, qsols, joint_angles);
    cout << "number of solutions found: " << nsols;
    cout << endl << "duration:" << duration.count() << endl;
    */

    // TEST franka_J_ik_q4()
    /*
    cout << endl << "=======================================================" << endl;
    cout << "franka_J_ik_q4()" << endl;
    cout << "=======================================================" << endl;
    ROE = { 0.3522750000000002,0.8334980991082203,0.4256560000000001,-0.6332361000000003,-0.12262271641620001,0.764183,0.6891402000000002,-0.5387433117066003,0.48460420000000004 };
    r = { -0.26617644,0.31984145,0.76295964 };
    q4 = -2.3401949461109055;
    joint_angles = true;
    start = high_resolution_clock::now();
    nsols = franka_J_ik_q4(r, ROE, q4, Jsols, qsols, joint_angles);
    end = high_resolution_clock::now();
    duration = duration_cast<microseconds>(end - start);
    print_results_J(Jsols, qsols, joint_angles);
    cout << "number of solutions found: " << nsols;
    cout << endl << "duration:" << duration.count() << endl;
    */

    // TEST franka_J_ik_swivel()
//    /*
    cout << endl << "=======================================================" << endl;
    cout << "franka_J_ik_swivel()" << endl;
    cout << "=======================================================" << endl;
    O_T_EE = {-0.28893,-0.774153,-0.563211,0,-0.956799,0.253474,0.142433,0,0.0324947,0.580032,-0.813945,0,0.408563,-0.0423964,0.37532,1};
    F_T_EE = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0.02,1};
    O_T_F = return_O_T_F_from_O_T_EE(O_T_EE, F_T_EE);
    ROE = return_ROE_from_O_T_EE(O_T_F);
    r = return_r_from_O_T_EE(O_T_F);   
/*
q_1 = 0.410566
q_2 = -0.402605
q_3 = -0.60232
q_4 = -2.53534
q_5 = 0.456634
q_6 = 2.08885
q_7 = 1.47693
quat: [ -0.5638132, 0.7675202, 0.2353244, 0.1940358 ] // Copied from https://www.andre-gaschler.com/rotationconverter/
*/
    theta = -0.3490658503988659;
    joint_angles = true;
    start = high_resolution_clock::now();
    nsols = franka_J_ik_swivel(r, ROE, theta, Jsols, qsols, joint_angles, '8');
    end = high_resolution_clock::now();
    duration = duration_cast<microseconds>(end - start);
    print_results_J(Jsols, qsols, joint_angles);
    cout << "number of solutions found: " << nsols;
    cout << endl << "duration:" << duration.count() << "us" << endl;
//    */

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