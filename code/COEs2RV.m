function [R, V] = COEs2RV(h, e, theta, mu, angles)
% function takes inputs of angular momentum, eccentricity and true anomaly
% (IN DEGREES) to calculate the state vectors in the perifocal reference
% frame. 

Omega = deg2rad(angles(1));
i = deg2rad(angles(2));
w = deg2rad(angles(3));

% calculate position vector
R = (h^2/mu*1)/(1+e*cosd(theta))*(cosd(theta)*[1 0 0] + sind(theta)*[0 1 0])';

% calculate velocity vector
V = mu/h*(-sind(theta)*[1 0 0] + (e + cosd(theta))*[0 1 0])';

% rotate this stuff
C21 = rotationMatrixFinder(3,1,3,Omega,i,w);
R = C21*R;
V = C21*V;

end

