function [v] = t2v(t)
% Convert from vector form of pose to corresponding homogenous transformation
% v: 3x1 vector representing the robot pose [x; y; theta]
% t: 3x3 matrix representing the homogenous transformation

v = [t(1, 3); t(2, 3); acos(t(1, 1))];
end