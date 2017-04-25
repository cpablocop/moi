function Hadm=as1_ip_tranferfunction(C)
%% function Hadm=as1_ip_tranferfunction(C)
% Function to derive the transfer function of the admittance 
% 
% Inputs
% C         : structure containing all the coefficients (as different
%             fields) of the linearized inverted pendulum model
% Output
%  Hadm     : transfer function of the admittance of the linearized
%             inverted pendulum. Hadm is an object of type tf (obtained by
%             using the function tf. 

% Complete the code below
% Definition of the variables used:
syms s
C_q = C.q;
C_qdot = C.qdot;
C_tauExt = C.tauExt;

% Derivation of the transfer function:
s = tf('s');
Hadm = C.tauExt/(s^2 - s*C.qdot - C.q);

end
