function [labdaPf,labdaDf]=as1_ip_musclelength(param,q)
%% function [labdaPf,labdaDf]=as1_ip_musclelength(param,q)
% Function to derive a symbolic expression for the muscle length of the
% plantar and dorsal flexor. 
% 
% Inputs
% param     : structure containing all the required parameters (e.g. fmax,
%             k0, kd, etc)
% q         : symbolic variable for the joint angle
%
% Output
%  labdaPf  : symbolic expression for the muscle length of the plantar
%             flexor
%  labdaDf  : symbolic expression for the muscle length of the dorsal
%             flexor
%
% Written by : Edwin van Asseldonk
% Date       : February 10 2017

% Complete the code below

% Definition of the symbolic variables labdaPf and labdaDf and the
% parameters used:
syms labdaPf labdaDf
labdaPf0 = param.labdaPf0;
labdaDf0 = param.labdaDf0;
r0 = param.r0;

% Calculation of length of the plantar flexor for an angle q
labdaPf = labdaPf0 + q*r0;

% Calculation of length of the dorsal flexor for an angle q
labdaDf = labdaDf0 - q*r0;

end



