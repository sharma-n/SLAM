function [sigma_points, w_m, w_c] = compute_sigma_points(mu, sigma, lambda, alpha, beta)
% This function samples 2n+1 sigma points from the distribution given by mu and sigma
% according to the unscented transform, where n is the dimensionality of mu.
% Each column of sigma_points should represent one sigma point
% i.e. sigma_points has a dimensionality of nx2n+1.
% The corresponding weights w_m and w_c of the points are computed using lambda, alpha, and beta:
% w_m = [w_m_0, ..., w_m_2n], w_c = [w_c_0, ..., w_c_2n] (i.e. each of size 1x2n+1)
% They are later used to recover the mean and covariance respectively.

n = length(mu);
##sigma_points = zeros(n,2*n+1);
%scale = lambda + n;
scale = 3.0;
lambda = scale - n;
% TODO: compute all sigma points
[U, D, V] = svd(sigma); %[V,D] = eig(sigma);
D = sqrt(D);
##D = max(D, 0.05 * eye(n)); % prevent the uncertainty in landmark being too low (going to 0)

sigmasqr = U * D * V';... sqrtm(sigma); %chol(sigma); Chose chol becuase nicer numerical properties

murep = repmat(mu, 1, n);
sigma_points = [mu, murep + sigmasqr, murep - sigmasqr];   

% TODO compute weight vectors w_m and w_c
w_m = [lambda/scale; repmat(1/(2*scale), 2*n, 1)];
w_c = w_m;
w_c(1) = w_c(1) + 1 - alpha^2 + beta;

end
