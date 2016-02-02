% Determine the next state of the system from the previous
% one.
% The result is based on the non-linear model for the mobile
% inverted pendulum system.

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

function [ F ] = nextstate(X,U,model)

% dotX = f(X,t)
% X is the state vector [p,dotp,phi,dotphi,theta,dottheta]
% U is the state input [F, tau]
% model is a vector defined by [M,m,D,L,R,Iwa,Iwz,g]; 

% Extract variables
p = X(1);
dotp = X(2);
phi = X(3);
dotphi = X(4);
theta = X(5);
dottheta = X(6);

% Extract input vector
tau = U(1);
Fp = U(2);

% Extract model parameters
M = model(1);
m = model(2);
D = model(3);
L = model(4);
R = model(5);
Iwa = model(6);
Iwz = model(7);
g = model(8);

if(abs(theta) < pi/2)

    F(1) = dotp;
    T1 = Fp + M*L*sin(theta)*dottheta^2 + M*L*sin(theta)^3*dotphi^2- M*g*sin(theta)*cos(theta);
    T2 = 2*m + M*sin(theta)^2 + 2*Iwa/R^2 ;
    F(2) = T1/T2;
    
    F(3) = dotphi;
    T1 = (tau+2*M*L^2*sin(theta)*cos(theta)*dottheta*dotphi - M*L*sin(theta)*...
        dotphi*dotp);
    T2 = Iwz+2*m*D^2+M*L^2*sin(theta)^2+2*D^2*Iwa/R^2;
    F(4) = T1/T2;
    
    F(5) = dottheta;
    T1 = Fp*cos(theta)+M*L*sin(theta)*cos(theta)*(dottheta^2-dotphi^2) - (2*m + M+2*Iwa/R^2)*(L*cos(theta)*dotphi^2+g)*sin(theta);
    T2 = M*L*cos(theta)^2-(2*m+M+2*Iwa/R^2)*L;
    F(6)=T1/T2;
else
    F = zeros(1,6);
end
F = F';
end

