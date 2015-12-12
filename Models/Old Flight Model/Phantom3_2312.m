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
    
%% Structure
    mFrame = 0.543; %vehicle frame mass, kg (QAV500 v2)
    mGear = 0.064; %vehicle landing gear mass, kg (CF legs)
    K = 4; %number of props
    mStructure = 1.216;
    
%% Power System
    mBatt = 1.245; %mass of battery, kg (Lumenier 10,000mAh)
    voltB = 15.2; %battery voltage, V (4s)
    mPower = 0;
    
%% Propulsion Parameters
    Dprop = 9.5*2.54/100; %prop diameter, m (Graupner Eprop)
    mProp = 0.015; %propeller mass , kg (Graupner Eprop)
    etaP = 0.6; %propeller efficiency
    mMotor = 0.110; %motor mass, kg (MN4010).
    etaM = 0.8; %motor efficiency
    mESC = 0.032; %ESC mass, kg (Lum ESC/BEC)
    mPropulsion = 0;
    
%% Flight Electronics Parameters
    mFC = 0.038; %flight computer mass, kg (Pixhawk)
    mRadio = 0.016; %radio mass, kg (3DR Radio)
    mGPS = 0.0168; %GPS/compass mass, kg (uBlox)
    mElec = 0;

%% Imagery Parameters
    mCamera = 0.074; %mass of imagery system, kg (GoPro Hero3 Black)
    mMount = 0.1; %mass of camera mount, kg (estimated)
    mVTX = 0.09; %mass of video transmitter, kg (3DR Kit)
    mImagery = 0;
    
%% Payload Parameters
    mServo = 0.025; %mass of servo/actuator, kg (linear actuator)
    mSPelec = 0.05; %mass of sensor package electronics, kg (current estimate)
    mSPstruct = 0.2; %mass of SP structure, kg (current estimate)
    mPayload = mServo + mSPelec + mSPstruct;
        
%% Initial Calculations
    mTotal = mStructure + mPower + mPropulsion + mElec + mImagery + mPayload;
    Fnet = mTotal*g/K; %net force from each motor, N
    Fnetg = mTotal*1000/K; %net mass carried by each motor, g
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
    