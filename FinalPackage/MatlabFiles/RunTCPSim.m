clear
clc
close all

if length(isgraphics(findall(0))) > 1
    delete(findall(0));
end

%Max timesteps for execution
k_max = 500;
%Log of time
time = zeros(1,k_max);
%Max velocity of human (only to simulate human motion)
maxP = 2;

load('paperspec4.mat')

TCPServer(k_max,time,maxP,freq,robots,humans,init_robot,init_human,input,map,inputNames,nodes)