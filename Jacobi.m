%% Function for Jacobian of Nonlinear measurement 'H'
function H = Jacobi(S_i, S_k, X_s,h_0)

S_i = [S_i, h_0];                                                           % UAV Initial Position
S_k = [S_k h_0];                                                            % UAV Actual Position
X_k = [X_s;h_0];                                                            % Predicted State

r_i = norm(X_k - S_i')^2;
r_k = norm(X_k - S_k')^2;

H = [(2 * (X_k(1) - S_i(1)) * r_k - 2 * (X_k(1) - S_k(1)) * r_i) / r_k^2,... % Jacobian matrix of Non-Linear Measurement  
     (2 * (X_k(2) - S_i(2)) * r_k - 2 * (X_k(2) - S_k(2)) * r_i) / r_k^2];
end