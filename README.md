# README: Fixed-wing UAV Project

## Overview

This repository encompasses two primary components:

1. **Autotuning PID Autopilot**: A self-adjusting Proportional-Integral-Derivative (PID) controller tailored for fixed-wing Unmanned Aerial Vehicles (UAVs).

2. **Conceptual Design Estimation Tool**: A software utility for estimating design parameters of modular aircraft based on inputs such as payload, endurance, and range.

## Repository Structure

- **Main_code/**: Contains the core implementation of the autotuning PID autopilot and the conceptual design estimation tool.

- **Snippets/**: Includes code snippets and auxiliary functions utilized within the main codebase.

- **Test/**: Comprises test scripts and datasets for validating and benchmarking the functionalities.

- **lib/**: Holds external libraries and dependencies required by the project.

- **UAV_Conceptual_variable.m**: A MATLAB script for defining variables and parameters pertinent to the conceptual design tool.

- **main.cpp**: The primary C++ source file implementing the autotuning PID controller.

## Getting Started

### Prerequisites

- **MATLAB**: Necessary for running the conceptual design estimation tool.

- **C++ Compiler**: Required to compile and execute the autotuning PID autopilot.

### Installation

1. **Clone the Repository**:

   ```bash
   git clone https://github.com/alanpaddy/Fixed-wing-UAV-Project.git
   ```


2. **Navigate to the Project Directory**:

   ```bash
   cd Fixed-wing-UAV-Project
   ```


3. **Compile the Autotuning PID Autopilot**:

   ```bash
   g++ -o autopilot main.cpp
   ```


4. **Run the Conceptual Design Tool**:

   Open `UAV_Conceptual_variable.m` in MATLAB and execute the script.

## Usage

### Autotuning PID Autopilot

After compiling, execute the `autopilot` binary. Ensure that the necessary input parameters, such as desired setpoints and initial conditions, are configured within the code or provided via input files.

### Conceptual Design Estimation Tool

Within MATLAB, run the `UAV_Conceptual_variable.m` script. Modify the input variables to reflect the desired payload, endurance, range, and other design parameters. The script will output estimated design characteristics of the modular aircraft.

## Contributing

Contributions are welcome! Please fork the repository, create a new branch for your feature or bugfix, and submit a pull request for review.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Acknowledgments

Special thanks to the open-source community and contributors who have provided valuable tools and libraries that facilitated this project.

---

*Note: This README provides a concise overview of the Fixed-wing UAV Project. For detailed documentation and advanced configurations, please refer to the respective code files and comments within the repository.* 
