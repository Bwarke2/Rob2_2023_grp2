function [output] = PID_controller(error,error_old,k_p,k_i,k_d)
%   PD controller
%   Simple PD controller
    d = error - error_old;          % Calculating derivative of error
    output = error*k_p + 0*k_i + d*k_d;      % PD-controller
end