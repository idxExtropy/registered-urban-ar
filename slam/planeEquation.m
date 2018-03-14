function [a, b, c, d] = planeEquation( n, p )
    a = n(1); b = n(2); c = n(3);
    d = a*p(1) + b*p(2) + c*p(3);
end

