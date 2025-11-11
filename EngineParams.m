
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
%% OPERATING CONDITIONS
% ========================================================================
Engine.rotation = 1000;  % rad/sec

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
