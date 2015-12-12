clear all; close all; clc;

Py = 30e6;

L = 32e-3;
m = 0.124;
g = 9.81;

r = 2.5e-3;
R = 5e-3;
k = R/sqrt(2);
A = 4*pi*(R^2-r^2);
mu = 0.08;

M = 2*m*g*L;
ps = M/A/k;
Fs = ps*A;
Fn = Fs/mu;
pn = Fn/A;
FOS = 30e6/pn;