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
    mF = 2.56; %vehicle frame mass, kg
    mP = 0.584;
    K = 4; %number of props
    
%% Motor/Prop Parameters
    Rprop = 5.5*2.54/100; %prop radius, m
    etaP = 0.4; %propeller efficiency
    etaM = 0.75; %motor efficiency
    voltM = 10; %motor voltage, V
    mM = 0; %motor mass, kg
        
%% Initial Calculations
    mV = mF + mP + K*mM; %total vehicle mass, kg
    Fnet = mV*g/K; %net force from each motor, N
    Aprop = pi*Rprop^2; %individual propeller area, m^2
    
%% Calculate Air Exit Velocity and Mass Flow
    Ve = sqrt(mV*g/K/rhoA/pi/Rprop^2); %exit velocity, m/s
    mdotA = rhoA*Aprop*Ve; %air mass flow rate per prop, kg/s
    
%% Calculate Power Required
    Pmotor = mdotA*Ve^2/2/etaP/etaM; %power required of each motor, W
    Ptotal = K*Pmotor; %total engine power required, W

%% Calculate Charge Required
    Itotal = Ptotal/voltM; %total current, A
    Qtotal = Itotal*tFlight*CtomAh; %total charge, A-hr
    Qreq = Qtotal/.8/.8;