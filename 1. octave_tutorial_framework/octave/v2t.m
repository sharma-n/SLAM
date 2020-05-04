function [t] = v2t(v)
% Convert from vector form of pose to corresponding homogenous transformation
% v: 3x1 vector representing the robot pose [x; y; theta]
% t: 3x3 matrix representing the homogenous transformation

t = [cos(v(3)) -sin(v(3)) v(1);
     sin(v(3))  cos(v(3)) v(2);
     0          0         1];
end