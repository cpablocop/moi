% function asl_ip_run
%% function as1_ip_run
% This is the main run MATLAB script for assignment 1, 1	Assignment 1: 
% "Stabilization of inverted pendulum model by intrinsic and reflexive 
% feedback"  of the Human Movement Control course at the University of Twente,
% The Netherlands.

% Instructions
% Run and step though this file to answer the questions in the assignment.
% You will need to complete the following functions:
%
%   as1_ip_musclelength.m
%   as1_ip_muscleforce.m
%   as1_ip_jointstiffness.m
%   as1_ip_jacobian.m
%   as1_ip_equationofmotion.m
%   as1_ip_solveequationofmotion.m
%   as1_ip_linearizesystem
%   as1_ip_transferfunction.m
% 
% Furthermore you need to complete the following simulink model
%
%   as1_ip_invertedpendulum  
%
% All of these functions have predefined inputs and outputs - do not change
% these! If you do so, we (the lecturers and teaching assistants) are not
% able to check and grade them.
%
% Academic integrity
% Although collaboration between students is allowed and even encouraged,
% you have to write and hand in your own code. We will act according to the
% academic integrity rules as set forth by the University of Twente.
%
% Written by : Edwin van Asseldonk
% Date       : February 10 2017

clc, clear all; %#ok<CLALL>

%% List of parameters defined in a structure
%Muscle and joint properties
param.fMax=6000;            % Maximal muscle force
param.k0=15000;             % Bias stiffness of each muscle
param.k1=150000;            % Activation dependent stiffness term
param.kd=0.1;               % Ratio between muscle stiffness and stiffness damping
param.labdaPf0=0.3;         % Length of plantar flexor muscle when pendulum is in upright postion
param.labdaDf0=0.4;         % Length of dorsal flexor muscle when pendulum is in upright postion
param.r0=0.05;              % Moment arm of plantar and dorsal flexor when pendulum is in upright postion

%Pendulum properties
param.lpend=1;              % Length of pendulum
param.m=70;                 % Mass of point mass on pendulum 

%Definition of equilibrium condition
param.eq.tauExt=0;          % Applied external torque
param.eq.q=0.1;             % Equilibrium pendulum angle [rad]
param.eq.qdot=0;            % Equilibrium pendulum angular velocity
param.eq.qRef=0.1;          % Equilibrium pendulum angular velocity
param.eq.coactivation=1;    % Amount of co-activation in equilibrium condition (0=no co-activation, 1=maximal co-activation)

%% Specify whether to use the default expressions
isLoadDefaultEquationPendulum=0;
isLoadDefaultLinearizedModel=0;
isLoadDefaultTransferfunction=0;

%% Define plot options
figpos = [0 0 550 550];
myblue = [0 0.4470 0.7410];
myred = [0.8500 0.3250 0.0980];
mygreen = [0.4660 0.6740 0.1880];

%% Define symbolics
syms q qdot uPf uDf  tauExt

%% Question I: Derive symbolic expression for muscle length and Jacobian
% Express the muscle lengths as function of the pendulum angle q
[labdaPf,labdaDf]=as1_ip_musclelength(param,q);
labda=[labdaPf;labdaDf];
%Derive the Jacobian to calculate muscle lengthening velocity from joint
%angular velocity 
JMu=as1_ip_jacobian(labda,q);

%Calculate the muscle length and jacobian for a pendulum angle of 0.1 rad
labdaPfSub=subs(labdaPf,{q},0.1);
labdaDfSub=subs(labdaDf,{q},0.1);
JmuSub=subs(JMu,{q},0.1);

%% Question IIa+b Derive equation of motion
%Calculate the reference muscle lengths
labdaPfRef=labdaPfSub; 
labdaDfRef=labdaDfSub; 

% Derive an expression for each of the muscle forces
muPf=as1_ip_muscleforce(param,labdaPf, labdaPfRef, uPf, q, qdot);
muDf=as1_ip_muscleforce(param,labdaDf, labdaDfRef, uDf, q, qdot);
mu=[muPf;muDf];

% Derive expression for q double dot. 
if isLoadDefaultEquationPendulum % load default expressions
    load as1_ip_default_equations_pendulum 
else % derive the expression yourself
    qdotdot=as1_ip_equationofmotion(param,q,mu,JMu,tauExt);
end

%% Question IIc+d Calculate the muscle activation for minimal and maximal co-activation

% Make a cell containing the symbolics that have to be replaced by their
% equilibrium value and a cell containing these values (see help subs for example)
symNameShortCell={tauExt, q, qdot}; %Short list of variables 
symValueShortArray=[param.eq.tauExt, param.eq.q, param.eq.qdot]; %Short list of variable values
symNameCell=[symNameShortCell {uPf,uDf}]; % Also include activation of plantar and dorsal flexor

% Calculate activation for minamal activation of plantar flexor
coactivation=0; %Set co-activation level
[uPfEq,uDfEq]=as1_ip_solveequationofmotion(param,qdotdot,coactivation,symNameShortCell,symValueShortArray); %Solve equation of motion
uEqMin=[uPfEq;uDfEq];
symValueArray=[symValueShortArray,uEqMin.']; %add calculated activation of array with values for symbols
muPfEq=double(subs(muPf,symNameCell,symValueArray)); % calculate the plantar muscle force
muDfEq=double(subs(muDf,symNameCell,symValueArray)); % calculate the dorsal muscle force
fprintf(1,'MINIMAL activation: muscle force in equilibrium condition muPf: %.2f, muDf: %.2f\n\n',muPfEq,muDfEq )  %#ok<*PRTCAL>
fprintf(1,'MINIMAL activation: muscle activation in equilibrium condition uPfEq: %.2f, uDfEq: %.2f\n\n',uPfEq,uDfEq )
% Calculate activation for maximal co-activation of plantar flexor
coactivation=1; %Set co-activation level
[uPfEq,uDfEq]=as1_ip_solveequationofmotion(param,qdotdot,coactivation,symNameShortCell,symValueShortArray); %Solve equation of motion
uEqMax=[uPfEq;uDfEq];
symValueArray=[symValueShortArray,uEqMax.']; %add calculated activation of array with values for symbols
muPfEq=double(subs(muPf,symNameCell,symValueArray)); % calculate the plantar muscle force
muDfEq=double(subs(muDf,symNameCell,symValueArray)); % calculate the dorsal muscle force
fprintf(1,'MAXIMAL coactivation: muscle force in equilibrium condition muPf: %.2f, muDf: %.2f\n\n',muPfEq,muDfEq )
fprintf(1,'MAXIMAL coactivation: muscle activation in equilibrium condition uPfEq: %.2f, uDfEq: %.2f\n\n',uPfEq,uDfEq )

%% Question III: Calculate the muscular contribution to the joint stiffness for minimal and maximal co-activation
[kJointMin, kMuMin]=as1_ip_jointstiffness(param,JMu,uEqMin);
[kJointMax, kMuMax]=as1_ip_jointstiffness(param,JMu,uEqMax);

fprintf(1,'Ankle joint stiffness MINIMAL %.1f MAXIMAL %.1f\n\n',kJointMin,kJointMax )

%% Question IV Linearize the system
if isLoadDefaultLinearizedModel
    load as1_ip_default_linearized_model 
else
    C=as1_ip_linearizesystem(qdotdot,{'q','qdot','uPf','uDf','tauExt'});
end

%Calculate coefficients for minimal co-activation
symValueArray=[symValueShortArray,uEqMin.']; %add calculated activation of array with values for symbols
C.min.q=double(subs(C.q,symNameCell,symValueArray));
C.min.qdot=double(subs(C.qdot,symNameCell,symValueArray));
C.min.uPf=double(subs(C.uPf,symNameCell,symValueArray));
C.min.uDf=double(subs(C.uDf,symNameCell,symValueArray));
C.min.tauExt=double(subs(C.tauExt,symNameCell,symValueArray));

%Calculate coefficients for maximal co-activation
symValueArray=[symValueShortArray,uEqMax.']; %add calculated activation of array with values for symbols
C.max.q=double(subs(C.q,symNameCell,symValueArray));
C.max.qdot=double(subs(C.qdot,symNameCell,symValueArray));
C.max.uPf=double(subs(C.uPf,symNameCell,symValueArray));
C.max.uDf=double(subs(C.uDf,symNameCell,symValueArray));
C.max.tauExt=double(subs(C.tauExt,symNameCell,symValueArray));

fprintf(1,'MINIMAL coactivation: Cq: %.2f, Cqdot: %.2f\n\n',C.min.q,C.min.qdot)
fprintf(1,'MAXIMAL coactivation: Cq: %.2f, Cqdot: %.2f\n\n',C.max.q,C.max.qdot)

%% Question V Derive the transferfunction
if isLoadDefaultTransferfunction
    load as1_ip_default_transferfunction 
else
    Hadm.min=as1_ip_tranferfunction(C.min);
    Hadm.max=as1_ip_tranferfunction(C.max);
end

%Plot bode diagrams for Hadm transfer functions
figure('Color','w','position',figpos)
bode(Hadm.min,'r',Hadm.max,'g')
legend({'Minimal activation','Maximal co-activation'},'position',[0.8 0.85 0.085 0.050])

% N Determine poles and zeros 
[z,p,k]=zpkdata(Hadm.min); %#ok<*ASGLU>
fprintf(1,'MINIMAL activation Zero %s - Poles %s\n\n', num2str(z{1}.'), num2str(p{1}.'))
[z,p,k]=zpkdata(Hadm.max);
fprintf(1,'MAXIMAL co-activation Zero %s - Poles %s\n\n', num2str(z{1}.'), num2str(p{1}.'))


%% Question VIa - Simulations in Simulink
if isLoadDefaultTransferfunction
    load as1_ip_default_transferfunction %#ok<*UNRCH>
end

%Set the parameters of the feedback loops to add to zero(this will allow
%running the model if you added the loops)
Gp=0; %#ok<*NASGU>
Gd=0;
TD=0; %Set to small value to prevent algabraic loop

% Initialize figure
figure('Color','w','position',figpos,'Name','Minimal and maximal co-activation')

% Set the variable in the LTI simulink block to Hadm.min
H=Hadm.min;
% Run the simulation
R=sim('as1_ip_invertedpendulum');
% Get the data from the simulation
yout=R.get('yout');
% Plot the data
as1_ip_plot(yout(:,1),yout(:,2),'VIa',myblue)

% Set the variable in the LTI simulink block to Hadm.max
H=Hadm.max;
% Run the simulation
R=sim('as1_ip_invertedpendulum');
% Get the data from the simulation
yout=R.get('yout');
% Plot the data
as1_ip_plot(yout(:,1),yout(:,2),'VIa',myred)
% Plot the step response
as1_ip_plot(yout(:,1),yout(:,3)/(10*max(yout(:,3))),'VIa','k')

legend({'Minimal activation','Maximal co-activation'},'Location','NorthEast')

%% Question VIb - Effect of Gp
% Set the parameters
GpArray=400:100:800;
nPar=length(GpArray);
Gd=0;
TD=0;

%Use model with minimal co-contraction
H=Hadm.min;

%Initialize figure
figure('Color','w','position',figpos,'Name','Effect of Gp')
lineArray=lines(nPar);

%Perform the simulations
for iPar=1:nPar
    Gp=GpArray(iPar);
   
    % Run the simulation
    R=sim('as1_ip_invertedpendulum');
    % Get the data
    yout=R.get('yout');
    % Plot the data    
    as1_ip_plot(yout(:,1),yout(:,2),'VIb',lineArray(iPar,:))
    if iPar==nPar
        as1_ip_plot(yout(:,1),yout(:,3)/(10*max(yout(:,3))),'VIb','k')
    end
end
legend((num2str(GpArray.')),'Location','NorthEast')     
  
%% Question VIc - Effect of Gd 
% Set the parameters
Gp=600; 
GdArray=0:20:100;
TD=0.001;
nPar=length(GdArray);

%Use model with minimal co-contraction
H=Hadm.min;

%Initialize figure
figure('Color','w','position',figpos,'Name','Effect of Gd')
lineArray=lines(nPar);

%Perform the simulations
for iPar=1:nPar
    Gd=GdArray(iPar);
   
    % Run the simulation
    R=sim('as1_ip_invertedpendulum');
    % Get the data
    yout=R.get('yout');
    % Plot the data    
    as1_ip_plot(yout(:,1),yout(:,2),'VIc',lineArray(iPar,:))
    
    if iPar==nPar
        as1_ip_plot(yout(:,1),yout(:,3)/(10*max(yout(:,3))),'VIc','k')
    end
end
legend((num2str(GdArray.')),'Location','NorthEast')    

%% Question VId - Effect of TD 
% Set the parameters
Gp=600; 
Gd=60;
TDArray=0:0.04:0.2;
nPar=length(TDArray);

%Use model with minimal co-contraction
H=Hadm.min;

%Initialize figure
figure('Color','w','position',figpos,'Name','Effect of TD')
lineArray=lines(nPar);

%Perform the simulations
for iPar=1:nPar
    TD=TDArray(iPar);
   
    % Run the simulation
    R=sim('as1_ip_invertedpendulum');
    % Get the data
    yout=R.get('yout');
    % Plot the data    
    as1_ip_plot(yout(:,1),yout(:,2),'VId',lineArray(iPar,:))
    if iPar==nPar
        as1_ip_plot(yout(:,1),yout(:,3)/(10*max(yout(:,3))),'VId','k')
    end
end
legend((num2str(TDArray.')),'Location','NorthEast')