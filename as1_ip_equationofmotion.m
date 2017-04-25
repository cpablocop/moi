function qdotdot=as1_ip_equationofmotion(param,q,mu,JMu,tauExt)
%% function qdotdot=as1_ip_equationofmotion(param,q,f,JMu,tauExt)
% Function which outputs the equation of motion of the inverted pendulum
% expressed as a function of the acceleration 
% 
% Inputs
%  param    : structure containing all the required parameters (e.g. fmax,k0, kd, etc)
%  q        : symbol for the joint angle
%  mu       : column vector containing the symboloic expression fo the muscle forces
%  Jmu      : Jacobian to get from joint to muscle space (labdadot=Jmu*qdot)
%  tauExt   : symbol for the external torque
%
% Output
%  qdotdot  : symbolic expression to calculate the angular acceleration of
%             the pendulum  
%
% Written by : Edwin van Asseldonk
% Date       : February 10 2017

% Complete the code below

%% Derive the equation of motion

% Define the parameters
g = 9.81;
r0 = param.r0;
m = param.m;
lpend = param.lpend;

% Derive the joint torque resulting from the muscle forces
tau = JMu'*(-mu);

% Derive the gravitational torque
taug = m*g*lpend*sin(q);

% Derive the equation of motion
qdotdot = (tau + taug + tauExt)/(lpend^2*m);
end