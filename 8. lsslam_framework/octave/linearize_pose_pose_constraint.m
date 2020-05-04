% Compute the error of a pose-pose constraint
% x1 3x1 vector (x,y,theta) of the first robot pose
% x2 3x1 vector (x,y,theta) of the second robot pose
% z 3x1 vector (x,y,theta) of the measurement
%
% You may use the functions v2t() and t2v() to compute
% a Homogeneous matrix out of a (x, y, theta) vector
% for computing the error.
%
% Output
% e 3x1 error of the constraint
% A 3x3 Jacobian wrt x1
% B 3x3 Jacobian wrt x2
function [e, A, B] = linearize_pose_pose_constraint(x1, x2, z)

  % TODO compute the error and the Jacobians of the error
  X1 = v2t(x1);
  X2 = v2t(x2);
  Z = v2t(z);
  e = t2v(Z\(X1\X2));
  
  theta_i = atan2(X1(2, 1), X1(1, 1));
  theta_ij = atan2(Z(2, 1), Z(1, 1));
  
  p1 = [-sin(theta_i), cos(theta_i)] * (x2(1:2)-x1(1:2));
  p2 = [-cos(theta_i), -sin(theta_i)] * (x2(1:2)-x1(1:2));
  p = [p1; p2];
  
  A = [-cos(theta_i+theta_ij), -sin(theta_i+theta_ij), [cos(theta_ij), sin(theta_ij)]*p; ...
       sin(theta_i+theta_ij), -cos(theta_i+theta_ij), [-sin(theta_ij), cos(theta_ij)]*p; ...
       0, 0, -1];
  B = eye(3);
  B(1:2, 1:2) = -1*A(1:2, 1:2);

end;
