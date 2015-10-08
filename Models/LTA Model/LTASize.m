%% PROJECT INFERNO -- CALCULATE LTA BALLOON RADIUS AND BLIMP LENGTH
% WRITTEN BY: Kevin Mulcair, 21 Sept 2015

clc; clear all; close all;

%% Constants
g = 9.81; %grav accel, m/s^2
RHe = 2077; %He specific gas const, J/kg/K
RH2 = 4124; %H2 specific gas const, J/kg/K

%% Parameters
h = 1600; %altitude, m -- 1600m = 1mi, Boulder local altitude
mPL = 0.1:0.1:10; %payload/stucture mass, kg

%% Standard Atmosphere for Ambient Conditions
[Tair, ~, Pair, rhoair] = atmosisa(h);

%% Find density of He & H2--assume ideal gas and Pgas = Pair
rhoHe = Pair/RHe/Tair; %density of helium, kg/m^3
rhoH2 = Pair/RH2/Tair; %density of hydrogen, kg/m^3

%% Find radius of spherical balloon
rHe = ( (3/4/pi) * (mPL/(rhoair-rhoHe)) ).^(1/3); %radius of helium balloon, m
rH2 = ( (3/4/pi) * (mPL/(rhoair-rhoH2)) ).^(1/3); %radius of hydrogen balloon, m

%% Find length of an ellipsoidal blimp ( L = 2a = 8b = 8c )
VHe = 4/3*pi*rHe.^3; %volume of helium, m^3
LHe = ( 96/pi*VHe ).^(1/3); %length of helium blimp, m
VH2 = 4/3*pi*rH2.^3; %volume of hydrogen, m^3
LH2 = ( 96/pi*VH2 ).^(1/3); %length of hydrogen blimp

%% Plot balloon radius as function of PL mass
figure(1)
hold on
grid on
title('Balloon Radius vs. Payload Mass')
xlabel('Mass [kg]')
ylabel('Radius [m]')
plot(mPL,rHe,'linewidth',2)
plot(mPL,rH2,'linewidth',2,'color','red')
legend('He','H_2','location','southeast')
hold off

%% Plot blimp length as function of PL mass
figure(2)
hold on
grid on
title('Blimp Length vs. Payload Mass')
xlabel('Mass [kg]')
ylabel('Length [m]')
plot(mPL,LHe,'linewidth',2)
plot(mPL,LH2,'linewidth',2,'color','red')
legend('He','H_2','location','southeast')
hold off

