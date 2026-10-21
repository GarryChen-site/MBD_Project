function [tsX, tsY] = scn_pedal(kind, u, T, par)
%SCN_PEDAL  Build raw ADC scenarios (timeseries of counts) for MIL tests.
%   kind : 'diag'   X and Y follow deflection u(t)  (normal driving)
%          'xonly'  X follows u(t), Y stays at centre (throttle-fault demo)
%          'yonly'  Y follows u(t), X at centre
%          'ygnd'   diag, but Y reads 0 counts (wire to GND)
%          'yvcc'   diag, but Y reads 4095
%   u    : function handle u(t) in [0,1] (deflection after dead band, i.e. n_i)
%   T    : stop time [s]
%   par  : init_params() struct (uses par.pedal calibration)
%   counts are the exact inverse of PedalCond:  a = c + db + n*(max - c - db)  (n>0), a = c (n=0)
Ts = par.Ts;  t = (0:Ts:T)';  n = u(t(:));  n = n(:);
p = par.pedal;
toCnt = @(n,c,db,mx) round(c + (n>0).*(db + n.*(mx - c - db)));
X = toCnt(n, p.cX, p.dbX, p.maxX);
Y = toCnt(n, p.cY, p.dbY, p.maxY);
rest = @(c) c*ones(size(t));
switch kind
    case 'diag'
    case 'xonly',  Y = rest(p.cY);
    case 'yonly',  Y = toCnt(n, p.cY, p.dbY, p.maxY);  X = rest(p.cX);
    case 'ygnd',   Y = zeros(size(t));
    case 'yvcc',   Y = 4095*ones(size(t));
    otherwise, error('unknown kind %s', kind);
end
tsX = timeseries(double(X), t);  tsY = timeseries(double(Y), t);
end
