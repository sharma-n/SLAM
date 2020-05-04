function [mu, sigma] = prediction_step(mu, sigma, u)
% Updates the belief concerning the robot pose according to the motion model,
% mu: 2N+3 x 1 vector representing the state mean
% sigma: 2N+3 x 2N+3 covariance matrix
% u: odometry reading (r1, t, r2)
% Use u.r1, u.t, and u.r2 to access the rotation and translation values

% TODO: Compute the new mu based on the noise-free (odometry-based) motion model
% Remember to normalize theta after the update (hint: use the function normalize_angle available in tools)
alpha = 0.9;
beta = 2;
kappa = 1;
n = 3;
lambda = alpha*alpha*(n+kappa)-n;
[sigma_pts, w_m, w_c] = compute_sigma_points(mu(1:3), sigma(1:3,1:3), lambda, alpha, beta);
theta = sigma_pts(3, :);
ang = theta + u.r1 + u.r2;
sigma_pts(1:2, :) += [(u.t*cos(theta+u.r1)); (u.t*sin(theta+u.r1))];

% TODO: Calculate the mean and covaiance of the sigma_points
mu(1:2) = sigma_pts(1:2,:)*w_m;
mu(3) = atan2(sin(ang)*w_m, cos(ang)*w_m);

% Motion noise
motionNoise = 0.1;
R3 = [motionNoise, 0, 0; 
     0, motionNoise, 0; 
     0, 0, motionNoise/10];
R = zeros(size(sigma,1));
R(1:3,1:3) = R3;

diff_sigma = sigma_pts - mu(1:3) * ones(1,length(w_m));
diff_sigma(3, :) = normalize_all_angles(diff_sigma(3,:));
sigma(1:3, 1:3) = diff_sigma * diag(w_c) * diff_sigma' + R3;

end
