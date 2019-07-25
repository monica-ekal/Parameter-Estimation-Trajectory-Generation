# Parmeter-Estimation-Trajectory-Generation

This repo contains code for performing nonlinear model predictive control (NMPC) that weights information gain to aid parameter estimation and improve system peformance. Information gain is measured using Fisher information of uncertain system parameters and appropriately weighting this information gain in the NMPC cost function. Full details can be found in the [paper](https://arxiv.org/abs/1906.02758) [Albee and Ekal, 2019].

The documentation for this repo is a work in progress. Code files are fairly densely commented for explanation--`NMPC_6DoF_main.m` is a good starting point.

##To run:

1. Install the ACADO Toolkit, following the instructions [here](https://acado.github.io/matlab_overview.html). Be sure to add ACADO to your path, which can be done in `NMPC_6DoF_main.m`.
2. Run `generate_NMPC_solver_*.m` in the ACADO_generator/ folder.
3. Run `generate_simulator_6DoF.m` in the ACADO_generator/ folder.
4. Run `NMPC_6DoF_main.m`.

## Repo summary:
* NMPC_6DoF_main.m - main file, which runs a simulation of the system dynamics,
	information-weighted NMPC, and an estimator

* Run_UKF_ - unscented Kalman filter class, called by the main file

* plot_results_general_no_estimates - a custom plotting script, called by the main file

* ACADO_generator - contains two MATLAB scripts that create the NMPC and dynamics simulation, respectively

* ACADO_generator/generate_NMPC_solver_6DoF - handles creating a weighting matrix, state vector, Fisher information terms, and interfacing with the ACADO NMPC API

* ACADO_generator/generate_simulator_6DoF - handles the 6DoF rigid body dynamics and interfacing with the ACADO API to generate a fast solver
