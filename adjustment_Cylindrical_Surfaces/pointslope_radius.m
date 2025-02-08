function [unit_vector, radius] = pointslope_radius(point)
% https://www.mathworks.com/matlabcentral/answers/424591-3d-best-fit-line#answer_341956
% 3xn matrix

xyz0 = mean(point,2);
A = point-xyz0;
[U,~,~] = svd(A, 'econ');
for i = 1:2
    d = U(:,i);
    t = d'*A;
    t1 = min(t);
    t2 = max(t);
    xzyl = xyz0 + [t1,t2].*d;

    p2 = xzyl(:, 2);
    p1 = xzyl(:, 1);
    vector = p2 - p1;
    
    if i == 1
        unit_vector = vector/norm(vector);
    elseif i == 2
        radius = (sqrt(vector(1)^2 + vector(2)^2 + vector(3)^2))/2;
    end
    
end