%% MN3508 Thrust Curve
clc; clear all; close all;

%% Atmospheric conditions
    rho = 1.04447; %air density at altitude, kg/m3
    rhoSea = 1.225; %density at sea level, kg/m3
    rhoRatio = rho/rhoSea; %ratio of densities
    
%% Power Usage
    Ivtx = 0.850*12/14.8;
    Ipixhawk = 0.5*5.3/14.8;
    Qdeploy = 0.1*5*5/14.8;
    
%% System Characteristics
    tFlight = 15*60;
    Vbatt = 14.8; %battery voltage, V
    CtomAh = 1000*0.000277778; %coulomb to amp-hr conversion
    Qbatt = 10000*0.8/CtomAh - Qdeploy; %Charge available, C
    thrustperaxis = 2455/4;
    Imax = Qbatt/tFlight*[1 1];

%% Calculate thrust curves
    rpm11 = [5300 6500 7200 8000 8500]; %11-inch RPMs
    A11 = [3.1,5.6,7.9,10.5,12.7]; %11-inch Amp draw
    thrust11 = [380 630 780 960 1110]; %11-inch thrust @ sea level, grams
    p11 = polyfit(rpm11,thrust11,2); %2nd-order fit for 11-inch
    pA11 = polyfit(A11,thrust11,2); % 2nd-order fit for 11-inch w/ Amps

    rpm12 = [4700 6300 6900 7500 8100]; %12-inch RPMs
    A12 = [3.8,7.4,10,13.5,16.1]; % 12-inch Amp draw
    thrust12 = [460 800 1000 1200 1360]; %12-inch thrust @ sea level, grams
    p12 = polyfit(rpm12,thrust12,2); %2nd-order fit for 12-inch
    pA12 = polyfit(A12,thrust12,2); % 2nd-order fit for 12-inch 2/ Amps
    
    rpm13p = 4200:10:7600; %rpm range for 13-inch
    A13p = 4.5:0.01:19.5; % 13-inch current draw
    thrust13p = 2*polyval(p12,rpm13p)*rhoRatio - polyval(p11,rpm13p)*rhoRatio;
        % 13-inch thrust as 12-inch plus delta from 11-inch
    thrust13pA = 2*polyval(pA12,A13p)*rhoRatio - polyval(pA11,A13p)*rhoRatio;
    
%% Calculate Design Characteristics
    hoverA = [3.1,19.5]; % Amp plot range
    hoverthrust = [thrustperaxis thrustperaxis]; %thrust at hover, grams
    
%% Calculate Max Characteristics
    MTOWthrust = max(thrust13p)*4*0.7*[1 1];

%% Plot
    RPMrange = [min(rpm13p) max(rpm13p)];

    figure(1)
    hold on
    grid on
    title('Child Drone Thrust vs. RPM','fontsize',14)
    xlabel('RPM','fontsize',14)
    ylabel('Thrust [grams]','fontsize',14)
    h5 = plot(rpm13p,thrust13p*4,'linewidth',2);
    h6 = plot(RPMrange,hoverthrust*4,'--','linewidth',2);
    h7 = plot(RPMrange,MTOWthrust,'--','linewidth',2);
    hL = legend([h5,h6,h7],'Thrust','Hover @ Design','Hover @ MTOW','location','northwest');
    set(hL,'fontsize',12)
    set(gca,'fontweight','bold')
    hold off
    
    thrustrange = 4*[min(thrust13p) max(thrust13p)];
    Ihover = (A13p(170)*4 + Ivtx + Ipixhawk)*[1 1];
    
    figure(2)
    hold on
    grid on
    title('Child Drone Current vs. Thrust','fontsize',14)
    xlabel('Thrust [grams]','fontsize',14)
    ylabel('Current [A]','fontsize',14)
    h11 = plot(thrust13pA*4,(A13p*4 + Ivtx + Ipixhawk),'linewidth',2);
    h12 = plot(thrustrange,Ihover,'--','linewidth',2);
    h13 = plot(thrustrange,Imax,'--','linewidth',2);
    hL2 = legend([h11,h12,h13],'Current','Current @ Hover','Endurance Requirement','location','northwest');
    set(hL2,'fontsize',10)
    set(gca,'fontweight','bold')
    hold off