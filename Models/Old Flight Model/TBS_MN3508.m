%% INFERNO Flight Model--Hovering Power Requirements
% Written By: Kevin Mulcair, 8 October 2015
clear all; close all; clc;

%% Constants
    g = 9.81; %grav acceleration, m/s^2
    CtomAh = 1000*0.000277778; %coulomb to amp-hr conversion

%% Flight Parameters
    alt = 1630; %altitude ASL, m
    [T,~,P,rhoA] = atmosisa(alt); %standard atmosphere
    tFlight = 15*60; %time of flight, sec
    
%% Vehicle Parameters
    mF = 0.414; %vehicle frame mass, kg
    mB = 1.245; %mass of battery, kg
    K = 4; %number of props
    voltB = 22.2; %battery voltage, V
    
%% Motor/Prop Parameters
    Dprop = 12*2.54/100; %prop radius, m
    etaP = 0.6; %propeller efficiency
    etaM = 0.8; %motor efficiency
    mM = 0.1; %motor mass, kg
    
%% Payload Parameters
    mE = 0.08; %mass of electronics, kg
    mI = 0.185; %mass of imagery system, kg
    mP = 0.500; %mass of payload, kg
        
%% Initial Calculations
    mV = mF + mB + mE + mI + mP + K*mM; %total vehicle mass, kg
    Fnet = mV/K*1000; %net force from each motor, N
    Aprop = pi*Dprop^2/4; %individual propeller area, m^2
    
%% Calculate Air Exit Velocity and Mass Flow
    v1 = sqrt(Fnet/Aprop/2/rhoA); %induced velocity, m/s
    
%% Calculate Power Required
    Pmotor = Fnet*v1/etaP/etaM; %power required of each motor, W
    Ptotal = K*Pmotor; %total engine power required, W

%% Calculate Charge Required
    Itotal = Ptotal/voltB; %total current, A
    Qtotal = Itotal*tFlight*CtomAh; %total charge, A-hr
    Qreq = Qtotal + Qtotal*.25;
    