# Readme File For Linear-MPC-Controller-for-Vehicle-Lateral-Dynamics-using-CasADI

## Program Overview
This MATLAB program simulates the lateral dynamics of a vehicle and solves a linear optimization problem to control the vehicle's movements in a planned trajectory. The optimization problem is solved using the qpOASES solver provided by the CasADi framework. The controller aims to minimize the tracking error relative to a predefined reference trajectory while adhering to the constraints on the vehicle's state variables and changes in control input.

## Vehicle Model
The vehicle is modeled as a 4-state system, with the states being lateral velocity (vy), yaw rate (r), lateral displacement (Y), and yaw angle (ψ). The system's control input is the steering angle (δ).

## Controller
The controller utilizes Model Predictive Control (MPC) with a prediction horizon of `Np = 10` steps and a control horizon of `Nc = 5` steps. The control sequence that minimizes the deviation from the reference trajectory and the control effort is determined by the controller. The tracking error is calculated by finding the difference between the actual vehicle state and a nonlinear reference trajectory specified by hyperbolic tangent functions (tanh).

## Constraints
The controller respects constraints on the yaw angle (ψ), lateral displacement (Y), and change in steering angle (δ). The constraints are designed to ensure that the vehicle does not exceed realistic physical limits. 

## Linear Optimization Problem
The controller sets up and solves a quadratic programming (QP) problem at each time step, looking for the optimal control sequence that minimizes a cost function consisting of tracking errors and control effort, subject to physical and system constraints. The qpOASES solver from the CasADi package is used to solve this QP problem efficiently.

## Simulation
The simulation loop sequentially updates the vehicle state and the reference trajectory. It uses the first control input from the MPC problem solution at each time step. The solver is then invoked again with updated initial conditions for the next step, and this process is repeated until the end of the simulation time, which is set to 15 seconds.

## Outputs
The program produces two figures upon execution. The first figure illustrates the actual and reference lateral displacements (Y and Y^r) as functions of the longitudinal displacement (X). The second figure portrays the actual and reference yaw angles (ψ and ψ^r) as functions of the longitudinal displacement (X). 

## Running the Program
MATLAB and the CasADi package are required to run this program. The entire script should be run to initiate the simulation, and the results will be automatically plotted at the end of the simulation.

## Program Limitations
This program assumes a constant vehicle velocity throughout the simulation. Any variation in velocity is not accounted for in the current version. Additionally, the reference trajectory is hardcoded in the program, so altering it will require modifications in the code.
