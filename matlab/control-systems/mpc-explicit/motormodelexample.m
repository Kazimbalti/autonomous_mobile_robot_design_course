function [sys, tau] = mpcmotormodel
% MPCMOTORMODEL plant data for DC-motor with elastic shaft
%
%   Reference: 
%
%   [1] A. Bemporad, "Reference Governors: On-Line Set-Point Optimization
%   Techniques for Constraint Fulfillment", Ph.D. dissertation, DSI,
%   University of Florence, Italy, 1997.

%   Copyright 1986-2014 The MathWorks, Inc.

%Parameters (MKS)
%------------------------------------------------------------------------------------------
Lshaft=1.0;      %Shaft length
dshaft=0.02;     %Shaft diameter
shaftrho=7850;   %Shaft specific weight (Carbon steel)
G=81500*1e6;     %Modulus of rigidity

tauam=50*1e6;    %Shear strength

Mmotor=100;      %Rotor mass
Rmotor=.1;       %Rotor radius
Jmotor=.5*Mmotor*Rmotor^2; %Rotor axial moment of inertia                      
Bmotor=0.1;      %Rotor viscous friction coefficient (A CASO)
R=20;            %Resistance of armature
Kt=10;           %Motor constant

gear=20;         %Gear ratio

Jload=50*Jmotor; %Load NOMINAL moment of inertia
Bload=25;        %Load NOMINAL viscous friction coefficient

Ip=pi/32*dshaft^4;               %Polar momentum of shaft (circular) section
Kth=G*Ip/Lshaft;                 %Torsional rigidity (Torque/angle)
Vshaft=pi*(dshaft^2)/4*Lshaft;   %Shaft volume
Mshaft=shaftrho*Vshaft;          %Shaft mass
Jshaft=Mshaft*.5*(dshaft^2/4);   %Shaft moment of inertia (???)

JM=Jmotor; 
JL=Jload+Jshaft;

%Input/State/Output continuous time form
%------------------------------------------------------------------------------------------
AA=[0             1             0                 0;
    -Kth/JL       -Bload/JL     Kth/(gear*JL)     0;
    0             0             0                 1;
    Kth/(JM*gear) 0             -Kth/(JM*gear^2)  -(Bmotor+Kt^2/R)/JM];
                
BB=[0;0;0;Kt/(R*JM)];

Hy=[1 0 0 0];
Hv=[Kth 0 -Kth/gear 0];

Dy=0;
Dv=0;

tau=tauam*pi*dshaft^3/16; %Maximum admissible torque

sys=ss(AA,BB,[Hy;Hv],[Dy;Dv]);
sys.InputUnit={'V'};
sys.OutputUnit={'rad';'Nm'};
