%% MN3508 Thrust Curve
clc; clear all; close all;

%% Atmospheric conditions
    rho = 1.04447; %air density at altitude, kg/m3
    rhoSea = 1.225; %density at sea level, kg/m3
    rhoRatio = rho/rhoSea; %ratio of densities
    
%% Airframe Characteristics
    Vbatt = 14.8; %battery voltage, V
    CtomAh = 1000*0.000277778; %coulomb to amp-hr conversion
    Qbatt = 7500/CtomAh;
    
%% Calculate Power Curves
    thrust11 = [380 630 780 960 1110]*rhoRatio; %11-inch thrust @ sea level, grams
    thrust11p = thrust11(1):10:thrust11(end);
    power11 = [45.88 82.88 116.92 155.40 187.96];
    p11power = polyfit(thrust11,power11,2);
    power11p = polyval(p11power,thrust11p);
    
    thrust12 = [460 800 1000 1200 1360]*rhoRatio; %12-inch thrust @ sea level, grams
    thrust12p = thrust12(1):10:thrust12(end);
    power12 = [56.24 109.52 148.00 199.80 238.28];
    p12power = polyfit(thrust12,power12,2);
    power12p = polyval(p12power,thrust12p);
    
    thrust13p = 450:10:1310;
    power13p = 2*polyval(p12power,thrust13p) - polyval(p11power,thrust13p);

    hoverthrust = [625 625];
    hoverY = [44 276];
    
%% Evaluate endurance
    Ihover = 4*power13p(thrust13p==630)/Vbatt;
    tFlight = Qbatt/Ihover/60;
    
    
%% Plot
    figure(1)
    hold on
    grid on
    title('MN3508 Power Curves','fontsize',14)
    xlabel('Thrust (grams)','fontsize',14)
    ylabel('Power (W)','fontsize',14)
%     plot(thrust11,power11,'linewidth',2);
    h2 = plot(thrust11p,power11p,'linewidth',2);
%     plot(thrust12,power12,'linewidth',2);
    h4 = plot(thrust12p,power12p,'linewidth',2);
    h5 = plot(thrust13p,power13p,'linewidth',2);
    h6 = plot(hoverthrust,hoverY,'--','linewidth',2);
    hL2 = legend([h2,h4,h5,h6],'11 in','12 in','13 in','Thrust @ Hover','location','southeast');
    hold off
    
    figure(2)
    hold on
    grid on
    title('MN3508 Current Curves','fontsize',14)
    xlabel('Thrust (grams)','fontsize',14)
    ylabel('Current (A)','fontsize',14)
    h2 = plot(thrust11p,power11p/Vbatt,'linewidth',2);
    h4 = plot(thrust12p,power12p/Vbatt,'linewidth',2);
    h5 = plot(thrust13p,power13p/Vbatt,'linewidth',2);
    h6 = plot(hoverthrust,hoverY/Vbatt,'--','linewidth',2);
    hL2 = legend([h2,h4,h5,h6],'11 in','12 in','13 in','Thrust @ Hover','location','southeast');
    hold off