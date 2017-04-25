function mu=as1_ip_muscleforce(param,labda, labdaRef, u, q, qdot)
%% function mu=as1_ip_muscleforce(param,labda, labdaRef, u, q, qdot)
% Function to derive a symbolic expression for the muscle forece
% 
% Inputs
% param     : structure containing all the required parameters (e.g. fmax,
%             k0, kd, etc)
% labda     : symbolic expressions for the muscle length of the concerned
%             muscle
% labdaref  : numeric value or the symbolic expression for the reference 
%             muscle lengths
% u         : symbolic variable for the muscle activation
% q         : symbolic variable for the joint angle
% qdot      : symbolic variable for the joint angular velocity
%
% Output
%  mu        : symbolic expression for the 
%
% Written by : Edwin van Asseldonk
% Date       : February 10 2017

% Complete the code below

fMax = param.fMax;
k0 = param.k0;
k1 = param.k1;
kd = param.kd;

% Calculate the muscular force:
stretch = labda - labdaRef;

% According to the chain rule, we can derive the velocity of the stretch:
stretchdot = diff(stretch, q)*qdot;
k = k0 + k1*u;
mu = fMax*u +  k*(stretch + kd*stretchdot);

end