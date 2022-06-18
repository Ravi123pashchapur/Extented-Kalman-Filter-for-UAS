%% Defining the function for Extended Kalman Filter.
function [x_state,P_cov,K_EKF_gain,innovation,h] = EKF_form(xy_1,xy_2,h_0,alpha,x_state_ini,P_cov_ini,F,G,Q,R)

%% Global declaration of variables
global X_s P_s

%% Initilization of estimate and error covarience.
X_s = x_state_ini;                                                          % Initial State Estimate
P_s = P_cov_ini;                                                            % Initial Covariance


%% Prediction Step for State Estimate and Error Covarience.

X_s = x_state_ini;                                                          % State Estimate
P_s = F*P_s*F' + G*Q*G';                                                    % Error covariance  

%% Correction Step for State Estimate and Error Covarience.

h = measure(xy_1,xy_2,X_s,h_0);                                               % Nonlinear measurement 

H = Jacobi(xy_1,xy_2,X_s,h_0);                                                % Jacobian of Nonlinear measurement

K_EKF_gain = (P_s * H')*(inv(H*P_s*H' +R));                                 % Kalman Gain calculation


%% Update Step for State Estimate and Error Covarience.

innovation = alpha-h;
x_state = X_s +K_EKF_gain*(alpha-h);                                        % Updated State Estimate
 
P_cov = (eye(size(K_EKF_gain*H))-K_EKF_gain*H)*P_s;                         % Updated Error Covarience

end



