
%% ========================================================================
% PHASE 1: MECHANICAL CORE - SINGLE CYLINDER SI ENGINE SIMULATION
% Compression-Ignition 4-Stroke Engine: Piston-Crank-Connecting Rod-Flywheel
% ========================================================================
% Purpose: Initialize baseline parameters for Simulink/Simscape model
% Note: This script sets up ALL geometric and physical parameters
% ========================================================================

clear all; close all; clc;


%% ENGINE GEOMETRIC PARAMETERS 
% ========================================================================

% Piston Params
Engine.L_piston = 0.060;    % m 
Engine.Bore = 0.0875;        % m
Engine.rho_piston = 2700;   % kg/m^3 

% Connecting Rod Params

Engine.L_rod = 0.234;       % m 
Engine.t_rod = 0.015;       % m
Engine.rho_rod = 7850;      % kg/m^3

% Flywheel Params

Engine.d_fwheel = 0.11;     % m
Engine.theta_0 = 0;          % Start at 0° (TDC). Change as needed.
Engine.t_fwheel = 0.015;
Engine.rho_fwheel = 7850;    % kg/m^3 
% CrankPin Params

Engine.cpin_radius = 0.007;         % m
Engine.cpin_length = 0.04;          % m
Engine.flywheel_offset_z = -0.02;   % offset of flywheel behind the main assembly (negative for behind)


%% ENGINE WORKING GEOMETRY

% Stroke length in meters
Engine.Stroke = Engine.d_fwheel;  % m

% Fly Wheel radius in meters
Engine.r_fwheel = Engine.Stroke / 2;  % m ,  half the stroke

% Bore-to-stroke ratio (B/S)
Engine.BS_ratio = Engine.Bore / Engine.Stroke;

% Rod-to-Flywheel radius  ratio (determines motion characteristics)
Engine.rod_fwheel_ratio = Engine.L_rod / Engine.r_fwheel;

%% ENGINE DISPLACEMENT AND VOLUME CALCULATIONS
% ========================================================================

% Piston cross-sectional area (m^2)
Engine.A_piston = (pi * Engine.Bore^2) / 4;

% Swept volume (displaced volume per stroke) in m^3
Engine.V_swept = Engine.A_piston * Engine.Stroke;

% Compression ratio (typically 8-12 for SI engines)
Engine.compression_ratio = 10;

% Clearance volume (dead volume at TDC) in m^3
Engine.V_clearance = Engine.V_swept / (Engine.compression_ratio - 1);

% Total cylinder volume at BDC (bottom dead center) in m^3
Engine.V_max = Engine.V_swept + Engine.V_clearance;

% Total cylinder volume at TDC (top dead center) in m^3
Engine.V_min = Engine.V_clearance;
%% COMBUSTION PARAMS & EQNS
%=========================================================================
% Fuel Params
Engine.fuel.CV = 44*10^6; % J/kg
Engine.fuel.rho = 740; %kg/m^3
Engine.combust.eta = 0.9; % unitless(is a ratio)
Engine.combust.theta = 5; %degrees at which the ignition signal is sent(at TDC)
Engine.combust.dur = 40; % degree for which the pressure pulse is present(plan is to keep P max at TDC, then make it decompose as theta goes frm 0 -> 40 in an exponential manner)
Engine.combust.afr = 12; % this is the air fuel ratio

%Combustion formulae
m_fuel = Engine.fuel.rho * Engine.V_max;
Engine.combust.E_comb = Engine.combust.eta * m_fuel * Engine.fuel.CV *(1/(Engine.combust.afr + 1));
%% 4. OPERATING CONDITIONS
% ========================================================================

% Engine speed in RPM (revolutions per minute)
Engine.RPM = 2000;  % 2000 RPM

% Engine speed in rad/s (used in simulations)
Engine.omega_min = (Engine.RPM * 2 * pi) / 60;  % rad/s

% Mean piston speed in m/s (useful for friction calculations)
Engine.V_piston_mean = 2 * Engine.Stroke * (Engine.RPM / 60);  % m/s

%% 5. FRICTION AND DAMPING PARAMETERS
% ========================================================================

% Friction coefficient between piston and cylinder wall (approximate)
Engine.mu_friction = 0.05;  % dimensionless (5% of normal force)

% Viscous damping coefficient for crankshaft bearings in N*m*s/rad
Engine.c_damping = 0.001;  % Small damping for smooth rotation

% Coulomb friction in crankshaft bearings in N*m
Engine.T_friction = 0.010;  % 0.01 N*m typical

%% 6. ATMOSPHERIC AND REFERENCE CONDITIONS
% ========================================================================

% Atmospheric pressure in Pa (reference pressure)
Engine.p_atm = 101325;  % 1 atm at sea level

% Reference temperature in K
Engine.T_ref = 298.15;  % 25 deg C

%% 7. PRESSURE WAVEFORM PARAMETERS (FOR LOOKUP TABLE INPUT)
% ========================================================================
% Phase 2 (Combustion) will generate these; Phase 1 uses external data

% Maximum cylinder pressure (peak combustion pressure) in Pa
Engine.p_max = 6.0e6;  % 60 bar (typical peak SI engine pressure)

% Number of crank angle points in pressure lookup table
Engine.n_pressure_points = 360;  % 1 degree per point

% Crank angles for one complete cycle (0-720 deg for 4-stroke)
Engine.theta_cycle = 0:(360/Engine.n_pressure_points):720;

%% 8. KINEMATICS: PISTON POSITION, VELOCITY, ACCELERATION FUNCTIONS
% ========================================================================
% These are analytical expressions for slider-crank kinematics
% Crank angle theta is in radians

% Pre-calculate frequently used parameters
r = Engine.r_fwheel;        % crank radius
l = Engine.L_rod;          % connecting rod length
n = Engine.rod_fwheel_ratio; % l/r ratio

% Generate crank angle array for one full cycle (0 to 2*pi rad)
theta_full = linspace(0, 2*pi, 720);

% Piston displacement as function of crank angle (in meters, from BDC)
% x(theta) = r * (1 - cos(theta)) + l - sqrt(l^2 - r^2*sin^2(theta))
Engine.x_piston = @(theta) r * (1 - cos(theta)) + l - sqrt(l.^2 - r^2 * sin(theta).^2);

% Piston velocity as function of crank angle (in m per degree, then m/s)
% First derivative of x with respect to theta
% v(theta) = r*omega*[sin(theta) + r*sin(theta)*cos(theta)/sqrt(l^2-r^2*sin^2(theta))]
Engine.v_piston = @(theta, omega) r * omega * ...
    (sin(theta) + (r * sin(theta) .* cos(theta)) ./ sqrt(l^2 - r^2 * sin(theta).^2));

% Piston acceleration as function of crank angle
% Second derivative of x with respect to theta
% a(theta) = r*omega^2*[cos(theta) + (term1 + term2)]
% (Complex expression with squared velocities)
Engine.a_piston = @(theta, omega) r * omega^2 * ...
    (cos(theta) + (r * (cos(theta).^2 - sin(theta).^2)) ./ ...
    sqrt(l^2 - r^2 * sin(theta).^2) - ...
    (r^2 * sin(theta).^2 .* cos(theta)) ./ (l^2 - r^2 * sin(theta).^2).^(3/2));

% Cylinder volume as function of crank angle
% V(theta) = V_clearance + A_piston * x(theta)
Engine.V_cylinder = @(theta) Engine.V_clearance + Engine.A_piston * Engine.x_piston(theta);

%% 9. TORQUE TRANSMISSION: PISTON FORCE TO CRANKSHAFT TORQUE
% ========================================================================
% Gas pressure P acts on piston area -> force F
% Force F is transmitted through connecting rod -> torque on crankshaft

% Mechanical advantage ratio (varies with crank angle)
% dV/dtheta = A_piston * dx/dtheta
% Torque = Force * (dV/dtheta) / A_piston  [for mechanical transmission]
% OR using kinematic chain:
% T_crank = F_piston * r * sin(theta) / (mechanical efficiency factor)

% Torque transmission function
% T_crank(theta, P) = P * A_piston * r * sin(theta) / 
%                     sqrt(l^2 - r^2*sin^2(theta))
Engine.torque_from_pressure = @(theta, P) ...
    P * Engine.A_piston * r * sin(theta) ./ sqrt(l^2 - r^2 * sin(theta).^2);

%% 10. INERTIA TORQUE (Dynamic effect of accelerating piston mass)
% ========================================================================
% Inertial torque opposes motion; essential for accurate dynamics

% T_inertia = -m_eq * a_piston * r * sin(theta) / sqrt(...)
Engine.inertia_torque = @(theta, omega) ...
    -Engine.m_piston_eq * Engine.a_piston(theta, omega) * r * sin(theta) ./ ...
    sqrt(l^2 - r^2 * sin(theta).^2);
%% VERIFICATION OF GRASHOF CONDITION (4-Bar Linkage Analysis)
% ========================================================================
% For slider-crank mechanism: Check if rod_crank_ratio > 1 (always true)
% Grashof condition: sum of longest and shortest links < sum of other two

grashof_check = Engine.rod_fwheel_ratio;
if grashof_check > 1
    fprintf('\nGrashof Check: PASSED - Valid slider-crank mechanism\n');
    fprintf('Rod-to-Crank Ratio: %.3f\n', grashof_check);
else
    warning('Grashof Check: FAILED - Invalid linkage ratio!');
end

%% 13. KINEMATIC ANALYSIS OUTPUTS (Pre-calculated)
% ========================================================================
% Calculate and store kinematic variables for one complete 4-stroke cycle

theta_analysis = linspace(0, 4*pi, 1440);  % 0.25 deg resolution for 4-stroke

Engine.analysis.theta = theta_analysis;
Engine.analysis.x_piston = Engine.x_piston(theta_analysis);
Engine.analysis.v_piston = Engine.v_piston(theta_analysis, Engine.omega_min);
Engine.analysis.a_piston = Engine.a_piston(theta_analysis, Engine.omega_min);
Engine.analysis.V_cylinder = Engine.V_cylinder(theta_analysis);

%% PARAMETER MASKS
E_comb    = Engine.combust.E_comb;
V_max     = Engine.V_max;
theta_ign = Engine.combust.theta;
dur       = Engine.combust.dur;

%% SUMMARY
% ========================================================================
fprintf('\n========== ENGINE PARAMETERS INITIALIZED ==========\n');

fprintf('\n\n=== READY FOR SIMULINK/SIMSCAPE SIMULATION ===\n\n');

% Store in workspace for use in Simulink
% Simulink will access these via base workspace
save engine_params.mat Engine;
