# GeoFIK: A fast and reliable geometric IK solver for the Franka arm

**GeoFIK** (**Geo**metric **F**ranka **IK**) is an analytical geometric solver for the inverse kinematics (IK) of the Franka arm. GeoFIK has the following advantages over other solvers:
* GeoFIK is ultra fast! It is an analytical solver, so the computational time in the order of tens of microseconds on standard laptops. This is much faster than numerical solvers and, according to our experiments, also faster than the algebraic analytical solvers generated by IKFast.
* Unlike other analytical solvers, GeoFIK provides four different ways to resolve the 1-DOF redundancy of the Franka arm. Functions that solve the IK using $$q\_7$$, $$q\_6$$, $$q\_4$$ or the swivel angle as free variables are provided. 
* Before computing the joint angles, GeoFIK finds the entire geometry of the arm by calculating the screw axes of the joints. Hence, the Jacobian matrices for each IK solution are returned at no significan extra computational cost.
* Due to its geometric nature, all possible solutions for the IK problem are found. We proved that the maximum numer is 8. Joint limits are checked before returning the solutions.
* All singularities are detected and handled efficiently. The user can provide emergency values of joint angles to resolve the singularity.

For detailed information about the implementation, tests and experiments, see the preprint of our paper [arXiv:2503.03992v1](https://arxiv.org/abs/2503.03992v1).

## Requirements

This version has been tested with GCC compiler for C++ 17. The functions use the Eigen library which can be downloaded [here](https://eigen.tuxfamily.org). This version was tested with Eigen 3.4.0.

## Usage

### World and end-effector frames

GeoFIK uses frame $$E$$ as end-effector frame, and frame $$O$$ as world frame, as shown in the picture below. In [franka_ros](https://frankaemika.github.io/docs/franka_ros.html) $$E$$ and $$O$$ are named `panda_hand_tcp` and `panda_link0`, respectively. 

![frames of the franka arm](frames.png) 

Some applications, especially when the gripper is removed, may use the flage frame 8 (`panda_link8_sc`) also called $$F$$ as end-effector frame. The transformation between this frame and frame $$E$$ is shown in the figure above and can be represented by the following transformation matrix $${}^{8}\_{E}$$𝐓: 
```
T8E = [[ 0.70710678  0.70710678  0.          0.        ]
       [-0.70710678  0.70710678  0.          0.        ]
       [ 0.          0.          1.          0.1034    ]
       [ 0.          0.          0.          1.       ]]
```
Therefore, when working with frame 8, given a desired pose, $${}^{O}\_{8}$$𝐓, the solver must be called with $${}^{O}\_{E}$$𝐓 = $${}^{O}\_{8}$$𝐓 $${}^{8}\_{E}$$𝐓.

### Functions provided

**Functions that only calculate joint angles**

All functions return the number of solutions found as an `unsigned int`.

---

```
unsigned int franka_ik_q7(const array<double, 3>& r,
                          const array<double, 9>& ROE,
                          const double q7,
                          array<array<double, 7>, 8>& qsols,
                          const double q1_sing = PI / 2)
```
Solves the IK with $$q\_7$$ as free variable, generates joint angles of solutions.
* `r` position of frame $$E$$ in world frame $$O$$.
* `ROE` rotation matrix of frame $$E$$ with respect to frame $$O$$ (row-first format).
* `q7` joint angle of joint 7 (radians). 
* `qsols` array to store 8 solutions.
* `q1_sing` [optional] emergency value of $$q\_1$$ in case of singularity at shoulder joints (type-1 singularity in our [paper](https://arxiv.org/abs/2503.03992v1))

---

```
unsigned int franka_ik_q6(const array<double, 3>& r,
                          const array<double, 9>& ROE,
                          const double q6,
                          array<array<double, 7>, 8>& qsols,
                          const double q1_sing = PI / 2,
                          const double q7_sing = 0)
```
Solves the IK with $$q\_6$$ as free variable, generates joint angles of solutions.
* `r` position of frame $$E$$ in world frame $$O$$.
* `ROE` rotation matrix of frame $$E$$ with respect to frame $$O$$ (row-first format).
* `q6`        joint angle of joint 6 (radians).
* `qsols` array to store 8 solutions.
* `q1_sing` [optional] emergency value of $$q\_1$$ in case of singularity at shoulder joints (type-1 singularity in our [paper](https://arxiv.org/abs/2503.03992v1))
* `q7_sing`   [optional] emergency value of q7 in case of type-2 singularity (see our [paper](https://arxiv.org/abs/2503.03992v1)).

---
 
```
unsigned int franka_ik_q4(const array<double, 3>& r,
                          const array<double, 9>& ROE,
                          const double q4,
                          array<array<double, 7>, 8>& qsols,
                          const double q1_sing = PI / 2,
                          const double q7_sing = 0)
```
Solves the IK with $$q\_4$$ as free variable, generates joint angles of solutions.
* `r` position of frame $$E$$ in world frame $$O$$.
* `ROE` rotation matrix of frame $$E$$ with respect to frame $$O$$ (row-first format).
* `q4`        joint angle of joint 4 (radians).
* `qsols` array to store 8 solutions.
* `q1_sing` [optional] emergency value of $$q\_1$$ in case of singularity at shoulder joints (type-1 singularity in our [paper](https://arxiv.org/abs/2503.03992v1))
* `q7_sing`   [optional] emergency value of q7 in case of type-2 singularity (see our [paper](https://arxiv.org/abs/2503.03992v1))

---

```
unsigned int franka_ik_swivel(const array<double, 3>& r,
                              const array<double, 9>& ROE,
                              const double theta,
                              array<array<double, 7>, 8>& qsols,
                              const double q1_sing = PI / 2,
                              const unsigned int n_points = 600)
```
Solves the IK with the swivel angle as free variable, generates joint angles of solutions. See our [paper](https://arxiv.org/abs/2503.03992v1) for the geometric definition of swivel angle used here. This is a numerical algorithm as no closed-form solution exists. However, since the solver only uses the strictly necessary geometric information in each iteration, the computational times on standard laptops range in the few hundreds of microseconds.
* `r` position of frame $$E$$ in world frame $$O$$.
* `ROE` rotation matrix of frame $$E$$ with respect to frame $$O$$ (row-first format).
* `theta`     swivel angle in radians (see paper for geometric defninition).
* `qsols` array to store 8 solutions.
* `q1_sing` [optional] emergency value of $$q\_1$$ in case of singularity at shoulder joints (type-1 singularity in our [paper](https://arxiv.org/abs/2503.03992v1))
* `n_points`  [optional] number of points to discretise the range of q7.

---

**Functions that only calculate joint angles and Jacobian matrices**

The following functions include the optional parameter `Jacobian_ee` which allows to specify an end-effector frame for the Jacobian. Note that this can be different to the end-effector frame of the IK which is _always_ $$E$$. When working with frames $$F$$, $$8$$ or $$6$$, this parameter allows the user to get the Jacobian in the desired frame without having to transform it manually, saving computational time.

---

```
unsigned int franka_J_ik_q7(const array<double, 3>& r,
                            const array<double, 9>& ROE,
                            const double q7,
                            array<array<array<double, 6>, 7>, 8>& Jsols,
                            array<array<double, 7>, 8>& qsols,
                            const bool joint_angles = false,
                            const char Jacobian_ee = 'E',
                            const double q1_sing = PI / 2)
```
Solves the IK with $$q\_7$$ as free variable, generates the joint angles and the Jacobians of all solutions.
* `r` position of frame $$E$$ in world frame $$O$$.
* `ROE` rotation matrix of frame $$E$$ with respect to frame $$O$$ (row-first format).
* `q7` joint angle of joint 7 (radians).
* `Jsols` array to store 8 Jacobian solutions.
* `qsols` array to store 8 solutions.
* `joint_angles` [optional] if `false` only Jacobians are returned.
* `Jacobian_ee` [optional] end-effector frame of the Jacobian (`'E'`, `'F'`, `'8'` or `'6'`). 
* `q1_sing` [optional] emergency value of $$q\_1$$ in case of singularity at shoulder joints (type-1 singularity in our [paper](https://arxiv.org/abs/2503.03992v1))

---

```
unsigned int franka_J_ik_q6(const array<double, 3>& r,
                            const array<double, 9>& ROE,
                            const double q6,
                            array<array<array<double, 6>, 7>, 8>& Jsols,
                            array<array<double, 7>, 8>& qsols,
                            const bool joint_angles = false,
                            const char Jacobian_ee = 'E',
                            const double q1_sing = PI / 2,
                            const double q7_sing = 0)
```
Solves the IK with $$q\_6$$ as free variable, generates the joint angles and the Jacobians of all solutions.
* `r` position of frame $$E$$ in world frame $$O$$.
* `ROE` rotation matrix of frame $$E$$ with respect to frame $$O$$ (row-first format).
* `q6`        joint angle of joint 6 (radians).
* `Jsols` array to store 8 Jacobian solutions.
* `qsols` array to store 8 solutions.
* `joint_angles` [optional] if `false` only Jacobians are returned.
* `Jacobian_ee` [optional] end-effector frame of the Jacobian (`'E'`, `'F'`, `'8'` or `'6'`). 
* `q1_sing` [optional] emergency value of $$q\_1$$ in case of singularity at shoulder joints (type-1 singularity in our [paper](https://arxiv.org/abs/2503.03992v1))
* `q7_sing`   [optional] emergency value of $$q\_7$$ in case of type-2 singularity (see our [paper](https://arxiv.org/abs/2503.03992v1)).

---

```
unsigned int franka_J_ik_q4(const array<double, 3>& r,
                            const array<double, 9>& ROE,
                            const double q4,
                            array<array<array<double, 6>, 7>, 8>& Jsols,
                            array<array<double, 7>, 8>& qsols,
                            const bool joint_angles = false,
                            const char Jacobian_ee = 'E',
                            const double q1_sing = PI / 2,
                            const double q7_sing = 0)
```
Solves the IK with $$q\_4$$ as free variable, generates the joint angles and the Jacobians of all solutions.
* `r` position of frame $$E$$ in world frame $$O$$.
* `ROE` rotation matrix of frame $$E$$ with respect to frame $$O$$ (row-first format).
* `q4`        joint angle of joint 4 (radians).
* `Jsols` array to store 8 Jacobian solutions.
* `qsols` array to store 8 solutions.
* `joint_angles` [optional] if `false` only Jacobians are returned.
* `Jacobian_ee` [optional] end-effector frame of the Jacobian (`'E'`, `'F'`, `'8'` or `'6'`). 
* `q1_sing` [optional] emergency value of $$q\_1$$ in case of singularity at shoulder joints (type-1 singularity in our [paper](https://arxiv.org/abs/2503.03992v1))
* `q7_sing`   [optional] emergency value of $$q\_7$$ in case of type-2 singularity (see our [paper](https://arxiv.org/abs/2503.03992v1)).

---

```
unsigned int franka_J_ik_swivel(const array<double, 3>& r,
                                const array<double, 9>& ROE,
                                const double theta,
                                array<array<array<double, 6>, 7>, 8>& Jsols,
                                array<array<double, 7>, 8>& qsols,
                                const bool joint_angles = false,
                                const char Jacobian_ee = 'E',
                                const double q1_sing = PI / 2,
                                const unsigned int n_points = 600)
```
Solves the IK with the swivel angle as free variable, generates the joint angles and the Jacobians of all solutions.
* `r` position of frame $$E$$ in world frame $$O$$.
* `ROE` rotation matrix of frame $$E$$ with respect to frame $$O$$ (row-first format).
* `theta`     swivel angle in radians (see paper for geometric defninition).
* `Jsols` array to store 8 Jacobian solutions.
* `qsols` array to store 8 solutions.
* `joint_angles` [optional] if `false` only Jacobians are returned.
* `Jacobian_ee` [optional] end-effector frame of the Jacobian (`'E'`, `'F'`, `'8'` or `'6'`). 
* `q1_sing` [optional] emergency value of $$q\_1$$ in case of singularity at shoulder joints (type-1 singularity in our [paper](https://arxiv.org/abs/2503.03992v1))
* `n_points`  [optional] number of points to discretise the range of $$q\_7$$.

---

**Other handy functions**

---

```
array<double, 7> J_to_q(const array<array<double, 6>, 7>& J,
                        const array<array<double, 3>, 3>& R,
                        const char ee = 'E')
```
Returns the joint angles given a Jacobian matrix and orientation of the end-effector frame.
* `J`  transpose of Jacobian matrix.
* `R` the rotation matrix of end-effector frame with respect to frame $O$. 
* `ee` [optional] name of end-effector frame (`'E'`, `'F'` or `'8'`).

---

```
array<array<double, 6>, 7> J_from_q(const array<double, 7>& q,
                                    const char ee = 'E');
```
Computes the Jacobian given the joint angles.
* `q`         joint angles, name of ee frame.
* `ee`        [optional] Name of end-effector frame (`'E'`, `'F'`, `'8'`, ...,`'1'`).

---

```
Eigen::Matrix4d franka_fk(const array<double, 7>& q, const char ee = 'E');
```
Forward kinematics function. Returns the transformation matrix of the specified end-effector frame with respect to world frame $$O$$.
* `q`         joint angles, 
* `ee`        name of ee frame (`'E'`, `'F'`, `'8'`, ...,`'1'`).

---

## Examples

The file `example_geofik.cpp` shows how to import GeoFIK, and how to call each of the solvers with case examples. For the fastest results, it is recommended to compile with optimization level 3, using a GCC compiler:
```
g++ example_geofik.cpp geofik.cpp -O3 -o example_geofik.exe
```

### Minimal example:

Consider the IK problem with end-effector position and orientation given by $${}^{O}\_{E}$$𝐓:

```
TOE = [[ 0.3522750000000002  0.8334980991082203  0.4256560000000001 -0.26617644]
       [-0.6332361000000003 -0.12262271641620001 0.764183            0.31984145]
       [ 0.6891402000000002 -0.5387433117066003  0.48460420000000004 0.76295964]
       [ 0.                  0.                  0.                  1.       ]]
```

and a desired value for the fourth joint angle $$q\_4$$ = -2.3402. Then, the following code calls the solver:

```
unsigned int nsols; 
array<double, 9> ROE = { 0.3522750000000002, 0.8334980991082203, 0.4256560000000001,
                        -0.6332361000000003, -0.12262271641620001, 0.764183,
                         0.6891402000000002, -0.5387433117066003, 0.48460420000000004 };
array<double, 3> r = { -0.26617644,0.31984145,0.76295964 };
double q4 = -2.3402;
array<array<double, 7>, 8> qsols;
nsols = franka_ik_q4(r, ROE, q4, qsols);
```

## Cite this work

This paper is currently under review. The ArXiv preprint can be found [here](https://arxiv.org/abs/2503.03992v1) and can be cited as:

* Lopez-Custodio PC, Gong Y, Figueredo LFC, “GeoFIK: A Fast and Reliable Geometric Solver for the IK of the Franka Arm based on Screw Theory Enabling Multiple Redundancy Parameters”, PREPRINT: arXiv:2503.03992v1 (2025)

```
@misc{lopezcustodio2025geofikfastreliablegeometric,
      title={GeoFIK: A Fast and Reliable Geometric Solver for the IK of the Franka Arm based on Screw Theory Enabling Multiple Redundancy Parameters}, 
      author={Pablo C. Lopez-Custodio and Yuhe Gong and Luis F. C. Figueredo},
      year={2025},
      eprint={2503.03992},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2503.03992}, 
}
```
