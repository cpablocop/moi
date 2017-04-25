function as1_ip_plot(t,data,question,color)
%% as1_ip_plot(t,data,question,color)
% This function plots the data for individual exercises. 
%
% Inputs
% t        ; time vector 
% data     : array with data, each signal in a seperate column
% qeustion : string indicating the question (e.g. VII, for question VII)
% color    : color ro be used in plotting. 
%
% Written by : Edwin van Asseldonk
% Date       : February 10 2017

desLineWidth=1.5;
switch question
    case 'VIa'
        plot(t,data,'Color',color,'LineWidth',desLineWidth)
        xlabel('Time [s]')
        ylabel('\deltaq [rad]')
        title('Minimal and maximal co-contaction without feedback')
        xlim([0 10])
        ylim([0 0.5])
        hold on
    case 'VIb'
        plot(t,data,'Color',color,'LineWidth',desLineWidth)
        title('Effect of Gp')
        xlabel('Time [s]')
        ylabel('\deltaq [rad]')
        xlim([0 20])
        ylim([0 0.11])
        hold on 
    case 'VIc'
        plot(t,data,'Color',color,'LineWidth',desLineWidth)
        title('Effect of Gd')
        xlabel('Time [s]')
        ylabel('\deltaq [rad]')
        xlim([0 20])
        ylim([0 0.11])
        hold on 
    case 'VId'
        plot(t,data,'Color',color,'LineWidth',desLineWidth)
        title('Effect of TD')
        xlabel('Time [s]')
        ylabel('\deltaq [rad]')
        xlim([0 20])
        ylim([-0.2 0.2])
        hold on   
    end
end
