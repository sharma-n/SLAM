function [mu, sigma, observedLandmarks, landmarks_order, curr_lms] = correction_step(mu, sigma, z, observedLandmarks, landmarks_order)
% Updates the belief, i. e., mu and sigma after observing landmarks, according to the sensor model
% The employed sensor model measures the range and bearing of a landmark
% mu: 2N+3 x 1 vector representing the state mean.
% The first 3 components of mu correspond to the current estimate of the robot pose [x; y; theta]
% The current pose estimate of the landmark with id = j is: [mu(2*j+2); mu(2*j+3)]
% sigma: 2N+3 x 2N+3 is the covariance matrix
% z: struct array containing the landmark observations.
% Each observation z(i) has an id z(i).id, a range z(i).range, and a bearing z(i).bearing
% The vector observedLandmarks indicates which landmarks have been observed
% at some point by the robot.
% observedLandmarks(j) is false if the landmark with id = j has never been observed before.

% Number of measurements in this time step
m = size(z, 2);
alpha = 0.6;
beta = 2;
kappa = 1;
n=3;
lambda = alpha*alpha*(n+kappa)-n;

landmarkIds = [z(:).id];
new_zs = z(observedLandmarks(landmarkIds)==false);
new_lms = landmarkIds(observedLandmarks(landmarkIds)==false);
for i=1:size(new_zs, 2)
  nMu = length(mu);
  lId = new_zs(i).id; range = new_zs(i).range; bearing = new_zs(i).bearing; % raw measurement
  [X, wm, wc] = compute_sigma_points([range; bearing], diag([0.01 0.01]), lambda, alpha, beta);
  % Transform range/bearing to Cartesian
  Range = X(1,:);
  Bearing = X(2,:);
  Dx = Range .* cos(Bearing + mu(3));
  Dy = Range .* sin(Bearing + mu(3));

  dx = Dx * wm;% Expected landmark position relative to robot position
  dy = Dy * wm;% = weighted average
  mu(nMu + (1:2)) = [mu(1) + dx; mu(2) + dy];

  dSigma = [Dx - dx; Dy - dy];
  % Stick the new weighted variance at the lower right corner of sigma
  sigma(nMu+(1:2),nMu+(1:2)) = dSigma * diag(wc) * dSigma';

  observedLandmarks(lId) = true; %add landmark to the map
  landmarks_order = [landmarks_order lId];
endfor

[X, w_m, w_c] = compute_sigma_points(mu, sigma, lambda, alpha, beta);
n_pts = size(X, 2);
[_,curr_lms] = ismember(landmarkIds,landmarks_order);
Dx = X(2*curr_lms+2, :) - ones(m,1) * X(1,:); % m x n_pts
Dy = X(2*curr_lms+3, :) - ones(m,1) * X(2,:); % m x n_pts
% Calcuate expected observation
sq = sqrt(Dx.^2 + Dy.^2);
expected_bearing = atan2(Dy, Dx) - ones(m,1) * X(3,:);
sq_avg = sq * w_m;
Z_avg = zeros(2*m, 1);
Z_avg(1:2:end) = sq_avg;
Z_avg(2:2:end) = atan2(sin(expected_bearing)*w_m, cos(expected_bearing)*w_m);

% sensor noise matrix
Q = eye(2*m)*0.01;

% correction step
dZ = zeros(2*m, n_pts);
dZ(1:2:end, :) = sq;
dZ(2:2:end, :) = expected_bearing;
dZ = dZ - Z_avg * ones(1,n_pts); % Subtract the mean of sample
dZ(2:2:end) = normalize_all_angles(dZ(2:2:end)); % Normalize the angles

S = dZ * diag(w_c) * dZ' + Q;

% Compute Sigma
dSigma = X - mu * ones(1, n_pts); % subtract the mean
dSigma(3,:) = normalize_all_angles(dSigma(3,:)); % normalize angles again

Sigma_xz = dSigma * diag(w_c) * dZ';

% Compute the Kalman gain
K = Sigma_xz / S;

% Update mu and sigma
innoZ = zeros(2*m,1);
innoZ(1:2:end) = [z(:).range]; % Form the innovation
innoZ(2:2:end) = [z(:).bearing];
innoZ = innoZ - Z_avg;
innoZ(2:2:end) = normalize_all_angles(innoZ(2:2:end));

mu = mu + K * innoZ;
mu(3) = normalize_all_angles(mu(3));
sigma = sigma - K * S * K';
end
