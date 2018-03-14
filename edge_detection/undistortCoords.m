function [u, v] = undistortCoords( u, v, fc, cc, kc)
    dx = (u - cc(1)) / fc(1);
	dy = (v - cc(2)) / fc(2); 
    x = dx; y = dy;

    for iter = 0:40
        r = sqrt(x * x + y * y);
        theta = atan(r);
        theta2 = theta * theta;
        theta_d = kc(3) + kc(4) * theta2;
        theta_d = kc(2) + theta_d * theta2;
        theta_d = kc(1) + theta_d * theta2;
        theta_d = (1 + theta_d * theta2) * theta;
        if (theta_d > 1e-8)
            scaling = r / theta_d;
            x = dx * scaling;
            y = dy * scaling;
        else
            break;
        end
    end
    u = x;
    v = y;
end

