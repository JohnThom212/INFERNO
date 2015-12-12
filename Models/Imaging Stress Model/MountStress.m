%% INFERNO Camera Mount Swivel Stress Model
clear all; close all; clc;

%% Material Properties
    sigmamax = 30e6; %tension yield stress of ABS P430, Pa

%% Swivel Properties
    r = 2.5e-3; %inner radius, m
    R = 5e-3; %outer radius, m
    k = R/sqrt(2); %average radius of friction, m
    A = 4*pi*(R^2-r^2); %frictional surface area, m2
    mu = 0.08; %minimum coeff of static friction for ABS

%% Generate Moment from Case
    L = 32e-3; %length of moment arm, m
    m = 0.124; %mass of case/camera, kg
    g = 9.81; %gravitational acceleration, m/s2
    M = 2*m*g*L; %moment from 2g load, N-m

%% Determine Stress
    sigmas = M/A/k; %shear stress to counteract moment, Pa
    Fs = sigmas*A; %net shear force, N
    Fn = Fs/mu; %net normal force required to generate shear force, N
    pn = Fn/A; %normal stress, N
    FOS = sigmamax/pn; %factor of safety