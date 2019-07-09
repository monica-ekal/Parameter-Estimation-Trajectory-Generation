6 DoF NMPC for states x,v,q,w, plus information weighting for estimating parameters.

Since we have x and v dependent on mass, and q and w dependent on Ixx,Iyy and Izz, the augmented state psi has 6 + 7*3, or 27 elements, discounting all the zeros that we dont want to waste time computing.


To run:
Run `generate_NMPC_solver_*.m` in ACADO_generator
Run `generate_simulator_6DoF.m` in ACADO_generator