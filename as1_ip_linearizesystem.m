function C=as1_ip_linearizesystem(qdotdot,symCell)
%% function C=as1_ip_linearizesystem(qdotdot,symCell)
% Function to linearize the non linear equation of motion by using the
% first order Taylor expansion. 
% 
% Inputs
% qdotdot   : symbolic expression to calculate the angular acceleration of 
%             the pendulum. This expression describes the dynamics of the 
%             inverted pendulum
% symCell   : cell containing symbolic variables. Qdotdot should be
%             linearized with respect to each of the variables in this cell
% Output
% C         : structure containing all the coefficients (as different
%             fields) of the linearized inverted pendulum model (e.g. C.q
%             contains the coefficient of deltaq of the linearized model)
%
% Written by : Edwin van Asseldonk
% Date       : February 10 2017

% Complete the code below

% Define the variables:
q = symCell{1};
qdot = symCell{2};
uPf = symCell{3};
uDf = symCell{4};
tauExt = symCell{5};

% According to Taylor's first order approximation:

C.q = diff(qdotdot,q);
C.qdot = diff(qdotdot,qdot);
C.uPf = diff(qdotdot,uPf);
C.uDf = diff(qdotdot,uDf);
C.tauExt = diff(qdotdot,tauExt);
end
