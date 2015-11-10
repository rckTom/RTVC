%% Abstract
%Author: Thomas Schmid
%Date: 04.10.2015
%Description: Motion analysis of a 2DOF gimbal

%% Sim
%Reference Frame = Gimbal Frame = Center Point Gimbal
%All Units mm

teta = pi/2;
phi = pi/2;

T21 = rotx(teta);
T10 = roty(phi);

vec = [0 1 0]';
T10*T21*vec
