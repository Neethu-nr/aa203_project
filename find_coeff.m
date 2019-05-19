function [CL, CD] = find_coeff(alpha)
    alpha = alpha * 180 /pi;
    if (alpha > 25)
        CL = NaN;
        CD = NaN;
    elseif (alpha > 20)
        CL = 1.596;
        CD = 0.583;
    elseif (alpha > 15)
        CL = 1.376;
        CD = 0.365;
    elseif (alpha > 10)
        CL = 1.102;
        CD = 0.184;
    elseif (alpha > 5)
        CL = 0.747;
        CD = 0.082;
    elseif (alpha > 0)
        CL = 0.365;
        CD = 0.039;
    else
        CL = NaN;
        CD = NaN;
    end
            
end