function [kQ, kMu]=as1_ip_jointstiffness(param,JMu,u)

%% function [kQ kMu]=as1_ip_jointstiffness(param,JMu,u)
% Function which outputs a matrix with the muscle stiffness and one with
% the joint stiffness
% 
% Inputs
%  param    : structure containing all the required parameters (e.g. fmax,k0, kd, etc)
%  JMu      : Jacobian (labdadot=Jmu*qdot) 
%  u        : column vector containing the muscular activation levels for
%             the different muscles
%
% Output
%  kQ        : Matrix with the joint sfiffnesses
%  kMu       : Matrix with the muscular stiffnesses.    
%
% Written by : Edwin van Asseldonk
% Date       : February 10 2017%

% Complete the code below

% Define the parameters:
r = param.r0;
k0 = param.k0;
k1 = param.k1;
uDf = u(2);
uPf = u(1);

% Derive the muscular stiffness:
kMu = [(k0+k1*uPf) 0; 0 (k0+k1*uDf)];

% Derive the joint stiffness from the muscular stiffness:
kQ = JMu'*kMu*JMu;

end
