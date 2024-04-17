function [rotMat] = quat_RotMat(eps,eta)

rotMat = (2*(eta^2) - 1)*eye(3) + 2*eps*eps' - 2*eta*crossComp(eps);

end