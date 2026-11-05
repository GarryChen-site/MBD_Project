function Td = loop_delay(Ts, Mavg)
%LOOP_DELAY  Td = 1.5*Ts + (Mavg-1)*Ts/2   (Stage 3 section 3.2)
if nargin < 2, Mavg = 1; end
Td = 1.5*Ts + (Mavg-1)*Ts/2;
end
