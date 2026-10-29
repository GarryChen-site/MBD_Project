function lambda = choose_lambda(tau, Td)
%CHOOSE_LAMBDA  lambda = max([6/pi*Td, 0.5*tau, 4*Td])   (Stage 3 section 3.3, discrete PM >= 60 deg rule)
lambda = max([6/pi*Td, 0.5*tau, 4*Td]);
end
