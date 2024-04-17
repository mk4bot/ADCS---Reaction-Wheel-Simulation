function [a_cross] = crossComp(a)

% a needs to be a 3 element vector

a_cross = [0 -a(3) a(2); a(3) 0 -a(1); -a(2) a(1) 0];

end