function [cylinder_model,inlier,outlier,sigma] = Least_squares_adjustment_of_Points_to_Cylindrical_Surfaces(point)
%The tool uses the least squares method to fit point cloud data to the cylindrical surface to obtain results 
% that are most representative of the cylinder. The adjustment result includes 7 parameters including cylindrical coordinates, 
% direction vector of cylindrical centerline, and radius. Finally, convert the parametric results to the coordinates of the start 
% and end points of the cylinder.

%https://www.mathworks.com/matlabcentral/fileexchange/116905-least-squares-adjustment-of-points-to-cylindrical-surfaces

point = double(point);

inputpoint = point; % n*3 matrix 
midvalue = min(inputpoint)+range(inputpoint)/2; % coordinate translate 
point = inputpoint - midvalue;

% sample = datasample(point, 10000, 'Replace', false); % sampling
% point = sample;

%% Initial value
% "a", "b", "c" as the direction vector of the cylinder centerline, 
% "d", "e", "f" is the coordinate of the perpendicular foot from origin to the centerline of the cylinder, 
% "r" for radius. 
[vec, r] = pointslope_radius(point'); % parameter a b c & r 
pos = min(point)+range(point)/2; % parameter d e f
X_ini = [vec; pos'; r];

%% Adjustment
iter = 0;
outlier = [];
correction = false;
blunder = true;
increment = []; 

while correction ~= true && blunder ~= false && iter < 1000
    [x_lowercase, X_hat, v, sigma, covariance] = adjust_cylinder_with_r(X_ini, point);
    increment = [increment; sigma, x_lowercase'];
    iter = iter + 1;
    fprintf('%d iteration completed. ', iter)
    if max(abs(x_lowercase)) < 1e-4 % approximate value increment check 
        correction = true;
        fprintf('approximate value increment reaches the threshold. ')
    else
        X_ini = X_hat;
        correction = false;
        fprintf('approximate value increment exceeds the threshold. \n')
    end
    
    while correction == true
        inlier = [];
        for j = 1:length(point) % residuals blunder detection 
            if abs(v(j)) > 3*sigma
                outlier = [outlier; point(j,:)];
            else
                inlier = [inlier; point(j,:)];
            end
        end

        if length(inlier) ~= length(point)
            point = inlier;
            X_ini = X_hat;
            correction = false; 
            fprintf('blunder exist. \n')
        else
            blunder = false;
            fprintf('blunder detection completed. \n')
            break
        end
    end
end

if iter == 1000
    error('This error message was made on purpose')
end

% disp('parameter a~f & r')
% disp(X_hat')
% fprintf('iterations: %d \n', iter)
% fprintf('sigma: %.4e m \n', sigma)
% posteriori = sqrt(diag(covariance)); % a posteriori mean-square error of unit weight
% disp('a posteriori mean-square error of unit weight')
% disp(posteriori')
% plot_approximationIncrement(increment)
% plot_residualsdistribution(v)

%%  Conversion result & plot
TF = isempty(outlier);
if TF == true
    outlier = [NaN,NaN,NaN];
end

% coordinate translate back 
inlier = inlier + midvalue;
outlier = outlier + midvalue;
X_hat(4:6) = X_hat(4:6) + midvalue';

[startp,endp] = params2endpoints_MY(inlier,X_hat(1:3),X_hat(4:6));


para = [startp, endp, X_hat(7)];
cylinder_model = cylinderModel(para); %注意：para由首尾两点坐标和半径组成
end