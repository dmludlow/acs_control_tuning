# ACS Controller Tuning MATLAB

## Overview
This project contains functionality to facilitate PD (Proportional-Derivative) controller gain tuning based on given initial and desired states, along with other physical properties and assumptions. The primary file in this repository is `PD_tuning.m`, which implements the tuning algorithms.

## Installation
To use this project, ensure you have MATLAB installed on your system. Simply download the repository and navigate to the project directory.

## Usage
1. Open MATLAB.
2. Navigate to the directory containing the `PD_tuning.m` file.
3. Call the functions defined in `PD_tuning.m` with the appropriate parameters for your specific application.

## Example
```matlab
% Example usage of PD_tuning.m
% Define initial and desired states
initial_state = [0; 0]; % Example initial state
desired_state = [1; 1];  % Example desired state

% Call the PD tuning function
[Kp, Kd] = PD_tuning(initial_state, desired_state);
```

## Contributing
Contributions are welcome! Please feel free to submit a pull request or open an issue for any enhancements or bug fixes.

## License
This project is licensed under the MIT License - see the LICENSE file for details.