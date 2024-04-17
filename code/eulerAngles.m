function [phi, theta, psi] = eulerAngles(mat)

phi = atan2(mat(2,3), mat(3,3));

theta = -asin(mat(1,3));

psi = atan2(mat(1,2), mat(1,1));

end