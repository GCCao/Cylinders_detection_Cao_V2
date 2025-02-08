function [cylinder_model,inlier,outlier,sigma] = Least_squares_adjustment_of_Points_to_Cylindrical_Surfaces_NEW(cylinderParam,point,magnification_sigma)
%The tool uses the least squares method to fit point cloud data to the cylindrical surface to obtain results 
% that are most representative of the cylinder. The adjustment result includes 7 parameters including cylindrical coordinates, 
% direction vector of cylindrical centerline, and radius. Finally, convert the parametric results to the coordinates of the start 
% and end points of the cylinder.

%magnification_sigma:用于控制内点，magnification_sigma通常为3，如果想要区分差别较小的圆柱，magnification_sigma可以降低到2

%https://www.mathworks.com/matlabcentral/fileexchange/116905-least-squares-adjustment-of-points-to-cylindrical-surfaces

% % figure
% % pcshow(point,[1,0,0],'MarkerSize',10); %显示点云数据

point = double(point); % n*3 matrix,圆柱拟合的参与点



inputpoint = point; % n*3 matrix 
midvalue = min(inputpoint)+range(inputpoint)/2; % coordinate translate 
point = inputpoint - midvalue;

% sample = datasample(point, 10000, 'Replace', false); % sampling
% point = sample;

%% Initial value
% "a", "b", "c" as the direction vector of the cylinder centerline, 
% "d", "e", "f" is the coordinate of the perpendicular foot from origin to the centerline of the cylinder, 
% "r" for radius. 

%[vec, r] = pointslope_radius(point'); % parameter a b c & r 

vec = cylinderParam(4:6)'; %这部分是xia增加的,直接用了种子点估算的结果。之所以替代原来的方式，是因为对于法兰外圆柱来说，pointslope_radius函数估算的不准
r   = cylinderParam(7);

pos = min(point)+range(point)/2; % parameter d e f
X_ini = [vec; pos'; r];

%% Adjustment
iter = 0;
outlier = [];
correction = false;
blunder = true;
increment = []; 

% if length(X_ini)<6
%     kkkkk=0;
% end

while correction ~= true && blunder ~= false && iter < 200 

    % iter
    % X_ini
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
            % if abs(v(j)) > 3*sigma
            if abs(v(j)) > magnification_sigma*sigma;
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

if iter == 200
    save('cylinderParam.mat', 'cylinderParam');
    point = inputpoint + midvalue;
    save('point.mat', 'point');  
    % error('This error message was made on purpose because the number of iterations has been exceeded')
    warning('the number of iterations has been exceeded！');

    cylinder_model = {};
    inlier =[];
    outlier=[];
    sigma = [];
    
    return;
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

A = para(1:3);
B = para(4:6);
AB_norm = norm(A-B);
if AB_norm == 0
    ee=0;
end

cylinder_model = cylinderModel(para); %注意：para由首尾两点坐标和半径组成


% if cylinder_model.Height<0.001
%     cylinder_model = {};
%     inlier =[];
%     outlier=[];
%     sigma = [];
% end

end