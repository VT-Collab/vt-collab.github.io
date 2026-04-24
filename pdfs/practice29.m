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

% learn using gradient descent
w = rand(4,1);
alpha = 0.01;
for idx = 1:1:1000
    w = w - alpha * X'*(X*w-U);
end
disp(w)

% learn using linear regression
w = inv(X'*X)*X'*U;
disp(w)