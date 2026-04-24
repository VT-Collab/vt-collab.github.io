clear
clc
close all

% the true value of w
w_star = [1; 0; 0; 0];

% gather training data
d = 10;
sigma = 0.1;
X = unifrnd(-1, 1, d, length(w_star));
U = X*w_star + normrnd(0, sigma, d, 1);

% learn the parameters
w = inv(X'*X)*X'*U;