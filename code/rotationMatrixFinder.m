function [C21] = rotationMatrixFinder(rotation1, rotation2, rotation3, psi, theta, phi)
% this is a function that takes a specified rotation sequence and ether
% symbolic variables for the euler angles or actual values for the euler
% angles and returns the corresponding rotation matrix for the specified
% euler angles or euler angle variables.
% Mike Kabot, written for Aero320, Oct. 2022.

% setup direction cosine matrices
xDCM = @(angle) [1 0 0; 0, cos(angle), -sin(angle); 0, sin(angle), cos(angle)];
yDCM = @(angle) [cos(angle), 0, -sin(angle);  0 1 0; sin(angle), 0, cos(angle)];
zDCM = @(angle) [cos(angle), -sin(angle), 0; sin(angle), cos(angle), 0; 0 0 1];

% use the rotation sequence to figure out which DCMs to use and where
if rotation1 == 1
    C1 = xDCM(psi);
elseif rotation1 == 2
    C1 = yDCM(psi);
elseif rotation1 == 3
    C1 = zDCM(psi);
else
    error('nope')
end

if rotation2 == 1
    C2 = xDCM(theta);
elseif rotation2 == 2
    C2 = yDCM(theta);
elseif rotation2 == 3
    C2 = zDCM(theta);
else
    error('nope')
end

if rotation3 == 1
    C3 = xDCM(phi);
elseif rotation3 == 2
    C3 = yDCM(phi);
elseif rotation3 == 3
    C3 = zDCM(phi);
else
    error('nope')
end

% once all the DCMs are determined find the product
C21 = C1*C2*C3;

end

