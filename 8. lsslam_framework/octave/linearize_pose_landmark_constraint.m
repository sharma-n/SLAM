% Compute the error of a pose-landmark constraint
% x 3x1 vector (x,y,theta) of the robot pose
% l 2x1 vector (x,y) of the landmark
% z 2x1 vector (x,y) of the measurement, the position of the landmark in
%   the coordinate frame of the robot given by the vector x
%
% Output
% e 2x1 error of the constraint
% A 2x3 Jacobian wrt x
% B 2x2 Jacobian wrt l
function [e, A, B] = linearize_pose_landmark_constraint(x, l, z)

  % TODO compute the error and the Jacobians of the error
  X = v2t(x);
  e = X(1:2,1:2)'*(l-x(1:2))-z;
  
  theta_i = atan2(X(2,1), X(1,1));
  l_x = l-x(1:2);

  A = [-cos(theta_i), -sin(theta_i), [-sin(theta_i), cos(theta_i)]*l_x; ...
       sin(theta_i), -cos(theta_i), [-cos(theta_i), -sin(theta_i)]*l_x];

  B = -1*A(1:2, 1:2);
end;
