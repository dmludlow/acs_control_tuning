%{
PD Controller Tuning Script for Spacecraft Attitude Control

This script simulates spacecraft attitude dynamics using quaternion math and
rigid body dynamics, and applies a PD controller for attitude regulation.
You can tune Kp and Kd and observe the response.

Conventions:
- Quaternions are [w x y z] (scalar first, MATLAB convention)
- Euler angles are [roll pitch yaw] in radians
- Inertia tensor is 3x3, units kg*m^2
%}

clear; clc;

%% ---------- Initial Conditions ----------

% Initial and desired Euler angles (roll, pitch, yaw)
initial_angle = deg2rad([0 0 0]);
desired_angle = deg2rad([30 30 30]);

% Initial and desired angular velocity
initial_w = [0 0 0]';
desired_w = [0 0 0]';

% Inertia tensor (example: 1U CubeSat)
I = [2.3e-3, -1e-5, 2e-5;
    -1e-5, 2.1e-3, -0.5e-5;
     2e-5, -0.5e-5, 2.25e-3];

%% ---------- Controller Gains  ----------
Kp = diag([2e-3 2e-3 2e-3]);   % Proportional gain (tune this)
Kd = diag([5e-4 5e-4 5e-4]);   % Derivative gain (tune this)
max_torque = 1e-3;

%% ---------- Simulation Parameters ----------
% Time step
dt = 0.001;

% Total simulation time [s]
T_total = 20;

steps = round(T_total/dt);

% Preallocate arrays
q_log = zeros(4, steps);
w_log = zeros(3, steps);
t_log = (0:steps-1)*dt;

%% ---------- Quaternion Initialization ----------
% Convert initial and desired Euler angles to quaternions [w x y z]
q0 = eul2quat(initial_angle, 'XYZ')';
q_desired = eul2quat(desired_angle, 'XYZ')';

% Create current state
q = q0;
w = initial_w;

%% ---------- Main Simulation Loop ----------
for k = 1:steps
    % Quaternion error (q_e = q_desired * q_current_conj)
    % Convert row -> column vector and back as needed.
    q_conj = [q(1); -q(2:4)];
    q_e = quatmultiply(q_desired', q_conj');
    q_e = q_e'; 

    % Use vector part of quaternion error
    q_e_vec = q_e(2:4);

    % Angular velocity error
    w_e = desired_w - w;

    % PD control law
    T = Kp*q_e_vec - Kd*w_e;

    % Torque saturation (clip each axis)
    T = max(min(T, max_torque), -max_torque);

    % Rigid body dynamics: w_dot = I^{-1} * (T - cross(w, I*w))
    w_dot = I \ (T - cross(w, I*w));

    % Integrate angular velocity (Euler's method)
    w = w + w_dot*dt;

    % Quaternion kinematics: q_dot = 0.5 * Omega(w) * q
    Omega = [ 0    -w(1) -w(2) -w(3);
              w(1)  0     w(3) -w(2);
              w(2) -w(3)  0     w(1);
              w(3)  w(2) -w(1)  0   ];
    q_dot = 0.5 * Omega * q;
    q = q + q_dot*dt;
    q = q / norm(q); % Normalize quaternion

    % Log data
    q_log(:,k) = q;
    w_log(:,k) = w;
end

%% ---------- Visualization ----------
% Convert quaternion log to Euler angles for plotting
euler_log = quat2eul(q_log', 'XYZ'); 

% Flip columns to maintain convention
euler_log = rad2deg(euler_log(:, [3 2 1])); 

figure (1);
plot(t_log, w_log(1,:), 'r', t_log, w_log(2,:), 'g', t_log, w_log(3,:), 'b');
xlabel('Time [s]'); ylabel('Angular Velocity [rad/s]');
legend('w_x','w_y','w_z'); title('Angular Velocity vs Time');

figure (2);
plot(t_log, euler_log(:,1), 'r', t_log, euler_log(:,2), 'g', t_log, euler_log(:,3), 'b');
xlabel('Time [s]'); ylabel('Euler Angles [deg]');
legend('Roll','Pitch','Yaw'); title('Euler Angles vs Time');