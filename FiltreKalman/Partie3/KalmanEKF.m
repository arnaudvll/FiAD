% -------------------------------------------------------------------------
%
% File : ExtendedKalmanFilterLocalizationTest.m
%
% Discription : Mobible robot localization sample code with
% Extended Kalman Filter (EKF)
%
% Environment : Matlab
%
% Author : Atsushi Sakai
% Contributor : Fabrice Jumel
% Copyright (c): 2014 Atsushi Sakai
%
% License : GPL Software License Agreement
% -------------------------------------------------------------------------
 
function [] = ExtendedKalmanFilterLocalizationTest()
 
close all;
clear all;
 
disp('Extended Kalman Filter (EKF) sample program start!!')
 
time = 0;
endtime = 300; % [sec]
global dt;
dt = 0.1; % [sec]
alpha=0.8    % Simple Filter Parameter
nSteps = ceil((endtime - time)/dt);
K =eye(4)
result.time=[];
result.xTrue=[];
result.xd=[];
result.xEst=[];
result.xsf=[]
result.z=[];
result.PEst=[];
result.u=[];
result.uTrue=[];
result.odomError=[];
result.measurmentError=[];
result.K=[];
 
% State Vector [x y yaw v]'
xEst=[0 0 0 0]';
 
% True State
xTrue=xEst;
 
% Dead Reckoning
xd=xTrue;

% Simple Filter
xsf=xTrue;
 
% Observation vector [x y yaw v]'
z=[0 0 0 0]';
 
% Covariance Matrix for motion
Q=diag([0.1 0.1 toRadian(1) 0.05]).^2; %Q=diag([0.1 0.1 toRadian(1) 0.05]).^2;
 
% Covariance Matrix for observation
R=diag([1.5 1.5 toRadian(3) 0.05]).^2;
 
% Simulation parameter
global Qsigma_real
Qsigma_real=diag([0.1 toRadian(20)]).^2; %[v yawrate]
global Rsigma_real
Rsigma_real=diag([1.5 1.5 toRadian(3) 0.05]).^2;%[x y z yaw v]

% 

PEst = eye(4);
 


% Main loop
for i=1 : nSteps
    time = time + dt;
    % Input
    u=doControl(time);
    % Odometry
    [xTrue,xd,un] = Odometry(xTrue, xd, u);

        %-------Simple Filter ------
        xsf = f(xsf, un);
        % ------ Kalman Filter --------
        % Predict
        xPred = f(xEst, un);
        F=jacobF(xPred, un);
        PPred= F*PEst*F' + Q
    %if 1
    if mod(i, 10) == 0 && i>0
        z = TakeMeasurement(xTrue);
        % Update Kalman
        H=jacobH(xPred);
        y = z - h(xPred);
        S = H*PPred*H' + R;
        K = PPred*H'*inv(S)
        xEst = xPred + K*y;
        PEst = (eye(size(xEst,1)) - K*H)*PPred;
        %Update Simple Filter
        xsf=xsf*alpha+z*(1-alpha);
        % No filter use sensor
        %xsf=z;
    else
        xEst= xPred;
        PEst =PPred;
        z=[NaN,NaN,NaN,NaN]';
    end
        % Simulation Result
        result.time=[result.time; time];
        result.xTrue=[result.xTrue; xTrue'];
        result.xd=[result.xd; xd'];
        result.xsf = [result.xsf; xsf'];
        result.xEst=[result.xEst;xEst'];
        
        result.z=[result.z; z']; 
        result.PEst=[result.PEst; diag(PEst)'];
        result.u=[result.u; un'];
        result.uTrue=[result.u; u'];
        result.odomError=[result.odomError; (u-un)'];
        result.measurmentError=[result.measurmentError; (xTrue-z)'];
        %result.K=[result.K K]
    
    

end

%?A?j???[?V???????
%movie2avi(mov,'movie.avi');
 
DrawGraph(result);

function ShowErrorEllipse(xEst,PEst)
%?????U?~???v?Z???A?\????????
Pxy=PEst(1:2,1:2);%x,y??????U????
[eigvec, eigval]=eig(Pxy);%??L?l???L?x?N?g????v?Z
%??L?l?????????C???f?b?N?X??T??
if eigval(1,1)>=eigval(2,2)
    bigind=1;
    smallind=2;
else
    bigind=2;
    smallind=1;
end

chi=9.21; %correpsond a 99%

%??~?`??
t=0:10:360;
a=sqrt(eigval(bigind,bigind)*chi);
b=sqrt(eigval(smallind,smallind)*chi);
x=[a*cosd(t);
   b*sind(t)];
angle = atan2(eigvec(bigind,2),eigvec(bigind,1));
if(angle < 0)
    angle = angle + 2*pi;
end

R=[cos(angle) sin(angle);
   -sin(angle) cos(angle)];
x=R*x;
plot(x(1,:)+xEst(1),x(2,:)+xEst(2))


function x = f(x, u)
% Motion Model
global dt;
 
F = [1 0 0 0
    0 1 0 0
    0 0 1 0
    0 0 0 0];
 
B = [
    dt*cos(x(3)) 0
    dt*sin(x(3)) 0
    0 dt
    1 0];
 
x= F*x+B*u;
 
function jF = jacobF(x, u)
% Jacobian of Motion Model
global dt;
 
jF=[
    1 0 0 0
    0 1 0 0
    -dt*u(1)*sin(x(3)) dt*u(1)*cos(x(3)) 1 0
     dt*cos(x(3)) dt*sin(x(3)) 0 1];

function z = h(x)
%Observation Model

H = [1 0 0 0
    0 1 0 0
    0 0 1 0
    0 0 0 1 ];
 
z=H*x;

function jH = jacobH(x)
%Jacobian of Observation Model

jH =[1 0 0 0
    0 1 0 0
    0 0 1 0
    0 0 0 1];

function u = doControl(time)
    %Calc Input Parameter
    % [V yawrate]

    % very simple constant control over time example 
    V=1.0; % [m/s]
    yawrate = 0; % 5 [deg/s]

    t_mod = mod(time, 5); % Temps modulé pour une période complète du carré
 
    % Calculer la commande en fonction du temps modulé pour suivre le carré
    if t_mod <= 1 % Avancer vers le haut
        V = 0;
        yawrate = 90; % Avancer en ligne droite
    end

    u =[ V toRadian(yawrate)]';
 

function [z, x, xd, un] = Observation(x, xd, u)
%Calc Observation from noise prameter
global Qsigma;
global Rsigma;

x=f(x, u);% Ground Truth
un=u+Qsigma*randn(2,1);%add Process Noise
xd=f(xd, un);% Dead Reckoning
z=h(x+Rsigma*randn(4,1));%Simulate Observation

function [z] = TakeMeasurement(x)
%Calc Observation from noise prameter
global Rsigma_real;
z=h(x+Rsigma_real*randn(4,1));%Simulate Noise on Measurement

function [x, xd, un] = Odometry(x, xd, u)
global Qsigma_real;
x=f(x, u);% Ground Truth
un=u+Qsigma_real*randn(2,1);%add Process Noise
xd=f(xd, un);% Dead Reckoning


function []=DrawGraph(result)
%Plot Result

figure(1);
x=[ result.xTrue(:,1:2) result.xEst(:,1:2) result.z(:,1:2)];
%x=[ result.xTrue(:,1:2) result.xsf(:,1:2) result.z(:,1:2)];
set(gca, 'fontsize', 16, 'fontname', 'times');
plot(x(:,5), x(:,6),'d','MarkerSize',10); hold on;
plot(x(:,1), x(:,2),'-.b','linewidth', 2); hold on;
plot(x(:,3), x(:,4),'r','linewidth', 2); hold on;
plot(result.xd(:,1), result.xd(:,2),'--k','linewidth', 4); hold on;
 
title('EKF Localization Result', 'fontsize', 16, 'fontname', 'times');
xlabel('X (m)', 'fontsize', 16, 'fontname', 'times');
ylabel('Y (m)', 'fontsize', 16, 'fontname', 'times');
%legend('GPS','Ground Truth','Dead Reckoning','EKF','Error Ellipse');
grid on;
axis equal;
plot(x(:,3), x(:,4),'r','linewidth', 2);
%figure(2)

% plot(result.time,result.xTrue(:,1),'-.b','linewidth', 2);hold on;
% plot(result.time,result.xEst(:,1),'r','linewidth', 2);hold on;
% plot(result.time,result.xd(:,1),'--k','linewidth', 2);hold on;
% plot(result.time,result.z(:,1),'d','MarkerSize',10);hold on;
% title('EKF Localization Result X', 'fontsize', 16, 'fontname', 'times');
% figure(3)
% plot(result.time,result.xTrue(:,2),'-.b','linewidth', 2);hold on;
% plot(result.time,result.xEst(:,2),'r','linewidth', 2);hold on;
% plot(result.time,result.xd(:,2),'--k','linewidth', 2);hold on;
% plot(result.time,result.xsf(:,2),'k','linewidth', 2);hold on;
% plot(result.time,result.z(:,2),'<','MarkerSize',10);hold on;
% title('EKF Localization Result Y', 'fontsize', 16, 'fontname', 'times');
% figure(4)
% plot(result.time,result.odomError(:,1),'*','MarkerSize',10);hold on;
% title('Odometry error over time R', 'fontsize', 16, 'fontname', 'times');
% figure(5)
% plot(result.time,result.odomError(:,2),'*','MarkerSize',10);hold on;
% title('Odometry error over time thetha', 'fontsize', 16, 'fontname', 'times');
% figure(6)
% plot(result.odomError(:,1),result.odomError(:,2),'*','MarkerSize',10);hold on;
% title('Odometry R and thetha', 'fontsize', 16, 'fontname', 'times');
% figure(7)
% plot(result.measurmentError(:,1),result.measurmentError(:,2),'*','MarkerSize',10);hold on;
% title('Mesarumeny Error X and Y', 'fontsize', 16, 'fontname', 'times');
% figure(8)
% plot(result.time,[(result.xTrue(:,1:2)-result.xd(:,1:2)) (result.xTrue(:,1:2)-result.xEst(:,1:2)) (result.xTrue(:,1:2)-result.z(:,1:2)) ],'*','MarkerSize',10);hold on;
% 
% title('Absolut Error X and Y', 'fontsize', 16, 'fontname', 'times');
% figure(9)
% plot(result.time,[(result.xTrue(:,1:2)-result.xEst(:,1:2)) (result.xTrue(:,1:2)-result.xsf(:,1:2))],'*','MarkerSize',10);hold on;
% 
% title('Simple Filter vs Kalman Filter Absolut Error X and Y', 'fontsize', 16, 'fontname', 'times');

disp(result.K)



function angle=Pi2Pi(angle)
%???{?b?g??p?x??-pi~pi????????????
angle = mod(angle, 2*pi);

i = find(angle>pi);
angle(i) = angle(i) - 2*pi;

i = find(angle<-pi);
angle(i) = angle(i) + 2*pi;


function radian = toRadian(degree)
% degree to radian
radian = degree/180*pi;

function degree = toDegree(radian)
% radian to degree
degree = radian/pi*180;