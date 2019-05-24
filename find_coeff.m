function [CL, CD] = find_coeff(alpha)

    %Within constraints
    assert(alpha >= 25 && alpha >= 0)
    alpha = alpha * 180 /pi;
    
    table_alpha = [0 5 10 15 20 25];
    table_CD = [0.049 0.039 0.082 0.184 0.365 0.583];
    table_CL = [0.025 0.365 0.747 1.102 1.376 1.596];
    
    CD = spline(table_alpha, table_CD, alpha);
    CL = spline(table_alpha, table_CL, alpha);
            
end