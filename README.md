# Quadcopter Control Simulation

## Project Overview
This project involves modeling, simulating, and controlling a quadcopter drone under various scenarios using MATLAB. The project demonstrates the quadcopter's behavior through both non-linear and linearized dynamic models.

## Getting Started
To use this project, ensure MATLAB is installed on your system. You can clone or download this repository to begin.

## Directory Structure
- **Q1**: Non-linear model simulations.
- **Q2**: Comparison between non-linear and linearized models.
- **Q3**: Implementation of a full-state feedback controller.
- **Q4**: Incorporates sensor noise and environmental disturbances into the simulation.

## Running Simulations
Each question's folder contains a specific MATLAB script `Sim_Quadcopter.m` for running simulations relevant to that question.

## Simulation Scenarios
- **Free Fall**: Simulates the quadcopter in a free fall with engines off.
- **Equilibrium**: Maintains a steady altitude and orientation.
- **Rotation at Constant Altitude**: Demonstrates spinning at a fixed altitude by adjusting the torque.
- **Linear vs Non-linear Model**: Examines behaviors under conditions that either favor or challenge the linear model.
- **Controlled Trajectory Flight**: Uses a state feedback controller to follow a designated path.
- **Disturbed Flight**: Simulates flight under realistic sensor noise and environmental disturbances.

## How to Run
1. **Basic Simulations (Q1 & Q2)**:
   - Navigate to the respective folder for the question.
   - Open and run `Sim_Quadcopter.m` in MATLAB.
   - Modify parameters within the script to see different behaviors.

2. **Advanced Control Simulations (Q3 & Q4)**:
   - Similar to the basic simulations, open and run the `Sim_Quadcopter.m` script in the respective directory.
   - These scripts integrate more complex features like controllers and observers into the simulations.

## Results Analysis
Plots and results are automatically saved in the `plots` folder for each question. Detailed observations can be made by examining these outputs.

## License
This project is licensed under the MIT License - see the `LICENSE` file for more details.

## Acknowledgments
- Special thanks to Andrew Gibiansky for foundational resources on quadcopter dynamics.
- Gratitude to course staff and teaching assistants for their invaluable support.
