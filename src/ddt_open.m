function dx = ddt_open(x, increment)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Based on ddt.m, filter22.m by RFK (Jan 1992), EJP (Aug 1998)
% Integrated single function version by University of Michigan helpful AI bot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if nargin < 2
    increment = 1;
end
if min(size(x)) > 1
    dx = arrayfun(@(i) ddt_open(x(:,i), increment), 1:size(x, 2), 'UniformOutput', false);
    dx = cell2mat(dx);
    return;
end
defaultFilterCoeffs = [-2 -1 0 1 2];
if size(x, 1) > 1 
    pct = x';
else
    pct = x;
end
numSides = 2;
numPoints = length(pct);
halfLen = ceil(length(defaultFilterCoeffs) / 2);
paddedPct = [pct zeros(1, halfLen)];

if numSides == 2
    y = filter(defaultFilterCoeffs, 1, paddedPct);
    dx = y(halfLen:(numPoints + halfLen - 1));
else
    dx = filter(defaultFilterCoeffs, 1, pct);
end
dx = -dx / (10 * increment);
m = length(dx);
dx(1) = (-21 * x(1) + 13 * x(2) + 17 * x(3) - 9 * x(4)) / (20 * increment);
dx(2) = (-11 * x(1) + 3 * x(2) + 7 * x(3) + x(4)) / (20 * increment);
dx(m) = (21 * x(m) - 13 * x(m-1) - 17 * x(m-2) + 9 * x(m-3)) / (20 * increment);
dx(m-1) = (11 * x(m) - 3 * x(m-1) - 7 * x(m-2) - x(m-3)) / (20 * increment);
if size(dx, 1) ~= size(x, 1)
    dx = dx';
end

end