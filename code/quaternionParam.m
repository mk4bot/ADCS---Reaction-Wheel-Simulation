function [epsilon,eta] = quaternionParam(mat)

% Finds the associated quaternion (epsilon, eta) parameterization for a rotation matrix
% mat

eta = sqrt(trace(mat) + 1)/2;

if eta == 0
    disp('Special Case: eta = 0!!!')
end

eps_1 = (mat(2,3) - mat(3,2))/(4*eta);
eps_2 = (mat(3,1) - mat(1,3))/(4*eta);
eps_3 = (mat(1,2) - mat(2,1))/(4*eta);

epsilon = [eps_1; eps_2; eps_3];

end