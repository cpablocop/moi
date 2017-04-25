function [uPfEq,uDfEq]=as1_ip_solveequationofmotion(param,qdotdot,coactivation,symNameCell,symValueArray)
%% function [muPfEq,muDfEq]=as1_ip_solveequationofmotion(param,ddq,coactivation,allParSyms,allParEq)
% Function to calculate the required muscular activation for the plantar
% and dorsal flexor. The required muscular activation are obtained by
% substiting all know parameters and variables such that one equation with
% one unknown variable remains which can be solved using solve. 
% NB to get the actual value, you might want to use eval(solve(...))
% 
% Inputs
% param     : structure containing all the required parameters (e.g. fmax,k0, kd, etc)
% qdotdot   : symbolic expression to calculate the angular acceleration of 
%             the pendulum. This expression describes the dynamics of the 
%             inverted pendulum%  
% coactivation : flag indicating whether to calculate the required muscle 
%                activation for minimal (co-)activation (coactivation =0)
%                or maximal co-activation (coactivation-1). 
% symNameCell : cell containing all the symbolic variables of which the
%               values are known
% symValueArray : array containing the values of all the symbolic variables
%                 in symNameCell. The order in symValueArray is equal to 
%                 symNameCell.  
%
% Output
%  uPfEq        : activation of the plantar flexor muscle for the given
%                 situation 
%  uPfEq        : activation of the dorsal flexor muscle for the given
%                 situation 
%
% Written by : Edwin van Asseldonk
% Date       : February 10 2017
% Complete the code below

% Define uPf and uDf as symbolic variables 
syms uPf uDf

% Derivation of the equation of motion according to the co-activation
% factor(This way the activatios will always be between 0 and 1):
qdotdotsubs = subs(qdotdot,symNameCell,symValueArray);
if coactivation==0
    [uPfEq,uDfEq] = solve([qdotdotsubs == 0],uDf==0,[uPf,uDf]);
elseif coactivation==1
    [uPfEq,uDfEq] = solve([qdotdotsubs == 0],uPf==1,[uPf,uDf]);
end;

end