% Linear and non-linear simulation for mobile inverted pendulum system,
% simulating noisy measurements and system discretization. The controller
% used here is optimal, determined using the LQR Technique.

% Copyright (C) 2015 Paulo Costa
% 
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% 
% GNU General Public License for more details.
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.
% 
% Contact: pcesar222@gmail.com


%% REQUIRED FILES: nextstate.m
clear all; clc; close all;

%% Parameters

% Robot
M = 1.46;                   % Pendulum mass 
m = 0.3;                    % Mass of each wheel
D = 0.08;                   % Distance from center to wheel
L = 0.11;                   % Distance from center to pendulum
R = 0.037;                  % Wheel radius
Iwa = m*R^2/2;              % Angular moment of wheels with respect to axis
Iwz = m*D^2;                % Angular moment of wheels with respect to z axis

max_torque = 0.77*2*0.5/2;  % Maximum torque available to the motor
                            % The 0.5 factor means we want to operate on half
                            % the stall torque

% Nature and life
g = 9.8;                % Gravity

% Simulation 
model = [M,m,D,L,R,Iwa,Iwz,g]; % Vector with all the constants that represent the model
h = 1e-4;                   %Simulation time step

% Sampling time
T = 20e-3;



%% State space matrices

A25 = -g*M/(2*m+2*Iwa/R^2);
A65 = g*(2*m+M+2*Iwa/R^2)/(2*m+2*Iwa/R^2)/L;
B22 = 1/(2*m+2*Iwa/R^2)/L;
B41 = 1/(Iwz + 2*m*D^2 + 2*D^2*Iwa/R^2);
B62 = -1/(2*m + 2*Iwa/R^2)/L;

Ac = [0,1,0,0,0,0; 
    0,0,0,0,A25,0;
    0,0,0,1,0,0;
    0,0,0,0,0,0;
    0,0,0,0,0,1;
    0,0,0,0,A65,0];
Bc = [0,0;
    0,B22;
    0,0;
    B41,0;
    0,0;
    0,B62];
Cc = [0,0,0,0,1,0];
Dc = 0;

% Create system
pendsys = ss(Ac,Bc,Cc,Dc);

% Discrete-time system, with ZOH discretization
pendsysd = c2d(pendsys,T);
Ad = pendsysd.a;
Bd = pendsysd.b;

% Uncoupled continuous:
A1c = [0,1,0,0;
     0,0,A25,0;
     0,0,0,1;
     0,0,A65,0];
B1c = [0;
    B22;
    0;
    B62];
A2c = [0, 1;
    0,0];
B2c = [0;B41];

% Uncoupled discrete
[A1d,B1d,C1d,D1d] = c2dm(A1c,B1c,zeros(1,4),zeros(1,1),T); % We don't really care about C,D
[A2d,B2d,C2d,D2d] = c2dm(A2c,B2c,zeros(1,2),zeros(1,1),T);

%% State-feedback controller design

% Position and theta control:
Q1 = eye(4); 
Q1(1,1) = 1000;  % prioritize x
Q1(3,3) = 1000; % prioritize theta
R1 =2.7;

K1 = dlqr(A1d,B1d,Q1,R1);


%% Simulation with estimated state equations

% We want to see the effect that the noise in the
% acelerometer/gyroscope has in the closed-loop system.

% Simulation time
tsim = 3.5; % seconds
simlength = floor(tsim/h);
timevec = 0:h:(simlength-1)*h;

% Initial conditions
p = 0 ; 
dotp = 0;
phi = 0;  phi = phi * pi/180;
dotphi = 0; 
theta = 5;  theta = theta * pi/180;
dottheta = 0;
F = 0;
tau = 0;

% Simulation storage vectors
statevec = zeros(6,simlength); % Captures the non-linear states 
stateveclin = zeros(6,simlength); % Captures the linear states
Uvec = zeros(2,simlength);
Uveclin = Uvec;

% Initial condition matrices
Xstate = [p,dotp,phi,dotphi,theta,dottheta]'; 
Xstatelin = Xstate;

U = [tau,F]'; 
Ulin = U;

% Kalman filter parameters

F = [1,-T; 0,1];
B = [T,0];
P = zeros(2,2);
Q = [0.001,0; 
    0,0.003];  % Sensor noise
R = 0.03;  %Process noise
H = [1 0];

% Storage vectors
pvecest = zeros(1,simlength);       
thetavecest = zeros(1,simlength);
dotthetavecest = zeros(1,simlength);
pmeasurevec = zeros(1,simlength);
thetameasurevec = zeros(1,simlength);
dotthetameasurevec = zeros(1,simlength);

% simulation to sampling time conversion factor
% This will help us simulate the controller update
toverh = T/h;
rateBias = 0;

% 0 if linear simulation, 1 if non-linear
simType = 0;
% 0 if linear plot, 1 if non-linear plot, 2 if both
plotType = 0;

for i=1:simlength
    
    % Capture states for simulation
    statevec(:,i) = Xstate;
    stateveclin(:,i) = Xstatelin;
    
    % Euler method
    Xstate = Xstate + h*nextstate(Xstate,U,model); 
    
    % Correct linear angle
    if(abs(Xstatelin(5)) < pi/2)
        Xstatelin = Xstatelin + h*(Ac*Xstatelin + Bc*Ulin);
    end
    
    % ============================================
    % ================  MAIN LOOP ================
    % ============================================
    if(mod(i-1,toverh) == 0)
        
        if simType == 0 
	        %Simulate sensor measurement with noise
            pmeasure = Xstatelin(1) +0.01*randn(1,1);
            dotpmeasure = Xstatelin(2)+0.02*randn(1,1);
            thetameasure = 180/pi*(Xstatelin(5)+0.01*randn(1,1));
            dotthetameasure = 180/pi*(Xstatelin(6)+0.01*randn(1,1));
        end
        if simType == 1 
        % Simulate sensor measurement with noise
        pmeasure = Xstate(1) +0.01*randn(1,1);
        dotpmeasure = Xstate(2)+0.02*randn(1,1);
        thetameasure = 180/pi*(Xstate(5)+0.01*randn(1,1));
        dotthetameasure = 180/pi*(Xstate(6)+0.01*randn(1,1));
        end
        
        %% Kalman Filtering
        
        % Estimate theta
        newRate = dotthetameasure - rateBias;
        
        if(i==1) 
            oldTheta = thetameasure;
        end
        thetaEst = oldTheta + T*newRate;
        
        % Estimate the covariance
        PnewEst = F*P*F' + Q; % Update predicted Covariance matrix
        
        % Innovation
        y = thetameasure - thetaEst;
        
        % Update innovation covariance
        S = P(1,1) + R;
        
        % Calculate Kalman gain
        K = (PnewEst*H'/S);
        
        % Update State
        newTheta = thetaEst + K(1)*y;
        newBias = rateBias + K(2)*y;
        
        % Update covariance
        P = (eye(2) - K*H)*PnewEst;
        
        % House keeping
        rateBias = newBias;
        oldTheta = newTheta;
        
        %% LQR control
        zstate = [pmeasure,dotpmeasure,pi/180*newTheta,pi/180*newRate];
        U1 = -K1*zstate';
        
        %% Update control signal
       
        U1lin = U1;
        
        U2 = 0;
        U2lin = 0;
        
        U = [U2;U1];
        Ulin = [U2lin;U1lin];
        
        % Correct torque on motors
        if(max_torque < abs(U(2)*R)) U(2) = max_torque/R*sign(U(2)); end
        if(max_torque < abs(Ulin(2)*R)) Ulin(2) = max_torque/R*sign(Ulin(2)); end
 
        
    end
    
    % Save measured values
    thetameasurevec(i) = thetameasure;
    dotthetameasurevec(i) = dotthetameasure;
   
    % Save estimated values
    thetavecest(i) = newTheta;
    dotthetavecest(i) = newRate;
    % Save control vector
    Uvec(:,i) = U;
    Uveclin(:,i) = Ulin;
    
end

phivec = statevec(3,:)';
pvec = statevec(1,:)';
thetavec = 180/pi*statevec(5,:)';

phiveclin = stateveclin(3,:)';
pveclin = stateveclin(1,:)';
thetaveclin = 180/pi*stateveclin(5,:)';


if plotType == 1 || plotType == 2
%% Non-linear simulation + control effort plot

    % Position
    figure(1);
    [temp,h1,h2] = plotyy(timevec,pvec,timevec,Uvec(2,:)*R); title('Non-linear simulation');
    xlabel('t(s)'); 
    set(h1,'linewidth',1.8);
    set(h2,'linewidth',1.8);
    axes(temp(1)); ylabel('position (m)');
    hold on; plot(timevec', zeros(1,simlength),'r--');
    axes(temp(2)); ylabel('control effort (Nm)');

    % Angle
    figure(2);
    [temp,h1,h2] = plotyy(timevec,thetavec,timevec,Uvec(2,:)*R);
    xlabel('t(s)'); 
    set(h1,'linewidth',1.8);
    set(h2,'linewidth',1.8);
    axes(temp(1)); ylabel('angle (degrees)');
    hold on; plot(timevec', zeros(1,simlength),'r--');
    axes(temp(2)); ylabel('control effort (Nm)');
end


if plotType == 0 || plotType == 2
%% Linear simulation + control effort plot

% Position
figure(3);
[temp,h1,h2] = plotyy(timevec,pveclin,timevec,Uveclin(2,:)*R); title('Linear simulation');
xlabel('t(s)'); 
set(h1,'linewidth',1.8);
set(h2,'linewidth',1.8);
axes(temp(1)); ylabel('position (m)');
hold on; plot(timevec', zeros(1,simlength),'r--');
axes(temp(2)); ylabel('control effort (Nm)');

% Angle
figure(4);
[temp,h1,h2] = plotyy(timevec,thetaveclin,timevec,Uveclin(2,:)*R);
xlabel('t(s)'); 
set(h1,'linewidth',1.8);
set(h2,'linewidth',1.8);
axes(temp(1)); ylabel('angle (degrees)');
hold on; plot(timevec', zeros(1,simlength),'r--');
axes(temp(2)); ylabel('control effort (Nm)');

end

%% Measuremets / Estimation plots

figure(5);
plot(timevec,thetameasurevec,timevec,thetavecest); xlabel('t(s)');
ylabel('Angle (degrees)');

figure(6);
plot(timevec,dotthetameasurevec,timevec,dotthetavecest); xlabel('t(s)');
ylabel('Angle (degrees)');

% Fix constants
K1 = 400*R/max_torque*K1;
FinalK = K1;
FinalK(3) = K1(3)*pi/180;
FinalK(4) = K1(4)*pi/180;
FinalK
