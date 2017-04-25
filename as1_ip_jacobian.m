function J=as1_ip_jacobian(labda,q)
%% function J=as1_ip_jacobian(labda,q)
% Function which outputs the Jacobian, containing the partial derivative of
% each elemenet in labda with respect to all variables declared in q
% 
% Inputs
%  labda    : column vector containing the symbolic expressions for the
%             different muscle lengts
%  q        : symbolic variable  for the joint angle
%
% Output
%  J        : Jacobian (labdadot=Jmu*qdot)      
%
% Written by : Edwin van Asseldonk
% Date       : February 10 2017%

% Complete the code below

%% Derive the Jacobian
J = diff(labda, q);
end
