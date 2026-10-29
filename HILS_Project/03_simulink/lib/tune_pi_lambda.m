function [Kp, Ki, Kb, info] = tune_pi_lambda(K, tau, lambda, Ts, TtFactor)
%TUNE_PI_LAMBDA  Lambda (IMC) PI tuning for G(s) = K/(tau s + 1)   (Stage 3 section 3.3)
%   Ti = tau,  Kp = tau/(K*lambda),  Ki = Kp/Ti = 1/(K*lambda),  Kb = 1/(TtFactor*tau)
%   Closed loop (ideal, no delay): T(s) = 1/(lambda s + 1), tr(10-90) = 2.2 lambda, ts(2%) ~ 3.9 lambda.
if nargin < 5, TtFactor = 1.0; end
Kp = tau/(K*lambda);
Ki = 1/(K*lambda);
Kb = 1/(TtFactor*tau);
info = struct('Ti',tau,'wc',1/lambda,'tr_expected',2.2*lambda,'ts_expected',3.9*lambda, ...
              'Ki_Ts',Ki*Ts,'lambda',lambda);
end
