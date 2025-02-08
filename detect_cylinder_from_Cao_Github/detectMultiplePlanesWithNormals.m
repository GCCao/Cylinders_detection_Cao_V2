function [planes, remainingPoints] = detectMultiplePlanesWithNormals(P, PlaneNormals, params)  
% detectMultiplePlanesWithNormals 检测点云中的多个平面并验证其合法性  
%  
% 输入参数：  
%   P: N×3 的点云矩阵  
%   PlaneNormals: N×3 的法向量矩阵，对应于点云 P  
%   params: 参数结构体（可选）  
%  
% 输出参数：  
%   planes: 检测到的平面结构体数组  
%   remainingPoints: 剩余未分类的点  

% figure(10)
% pcshow(P,[0.5,0.5,0.5],'MarkerSize',5); %显示点云数据


% 1. 参数初始化
if nargin < 3   
    % 设置参数
    params.distThreshold = 0.3;    % 距离阈值
    params.minPoints = 200;         % 最小点数
    params.maxPlanes = 100;          % 最大平面数
    params.confidence = 0.99;       % RANSAC 置信度
    params.maxIterations = 1000;    % RANSAC 最大迭代次数
    params.angleThreshold = 5;     % 法向量夹角阈值（度）
    params.invalidRatioThreshold = 0.6; % 无效点比例阈值
    params.maxIteration = 5000;    % 主循环最大迭代次数
end

% 2. 初始化输出
planes = struct('points',{},'normal',{},'d',{});
remainingPoints = P;  
remainingNormals = PlaneNormals;  
planeCount = 0;  
iterationCount = 0;  

% 3. 主循环：持续检测平面直到满足终止条件  
while size(remainingPoints,1) >= params.minPoints && ...  
      planeCount < params.maxPlanes && ...  
      iterationCount < params.maxIteration  

    iterationCount = iterationCount + 1 
    
    % 3.1 使用 RANSAC 检测单个平面  
    [planeModel, inlierIndices] = ransacFitPlane(remainingPoints,remainingNormals, params);  
    
    % 3.2 如果找到满足条件的平面  
    if ~isempty(planeModel) && length(inlierIndices) >= params.minPoints  
        % 获取平面上的点及其法向量  
        planePoints = remainingPoints(inlierIndices,:);
        planePointNormals = remainingNormals(inlierIndices,:);

        % figure
        % pcshow(P,[0.5,0.5,0.5],'MarkerSize',5); %显示点云数据
        % color1 = rand(1, 3);
        % hold on;
        % pcshow(planePoints,color1,'MarkerSize',20);


        % 计算主轴长度
        axesLengths = computePrincipalAxesLengths(planePoints);
        % 显示结果
        % fprintf('主轴长度分别为：\n');
        % fprintf('Axis 1: %.4f\n', axesLengths(1));
        % fprintf('Axis 2: %.4f\n', axesLengths(2));
        % fprintf('Axis 3: %.4f\n', axesLengths(3));

        isValidPlaneaxesLengths = 0;
        if (axesLengths(1)/axesLengths(2) <3)
            isValidPlaneaxesLengths = 1;

        else            
            % 将不合法平面的点保留在 remainingPoints 中  
            continue;  
        end

        isValidPlane =0;
        if isValidPlaneaxesLengths          
            % figure           
            % pcshow(P,[0.5,0.5,0.5],'MarkerSize',5); %显示点云数据
            % color1 = rand(1, 3);
            % hold on;
            % pcshow(planePoints,color1,'MarkerSize',20);

            % 检查平面的合法性(法向量正常的点的数量占比)
            isValidPlane = checkPlaneValidity(planePoints, planePointNormals, ...
                planeModel(1:3), params);
           % isValidPlane =1;
        else
             % 将不合法平面的点保留在 remainingPoints 中  
            continue;  
        end


        %%%%%%%%%
        isSymmetric =0;
        if isValidPlane

            % 1. 首先将点云投影到平面上
            [Q, transform] = projectPointsToPlane1(planePoints, planeModel);

            % 2. 检查点云分布的双轴对称性
            tolerance = 5; % 设置容差
            [isSymmetric, symmetryScores, symmetryAxes] = checkDualAxisSymmetry(Q, tolerance);
           %  [isSymmetric, symmetryScores, symmetryAxes] = checkDualAxisSymmetry_new(Q, tolerance);  
            % symmetryScores            
        end
        %%%%%%

        

        
        
        if isSymmetric

            % figure(10)           
            % pcshow(P,[0.5,0.5,0.5],'MarkerSize',5); %显示点云数据
            % color1 = rand(1, 3);
            % hold on;
            % pcshow(planePoints,color1,'MarkerSize',20);

            % 存储合法平面  
            planeCount = planeCount + 1;  
            planes(planeCount).points = planePoints;  
            planes(planeCount).normal = planeModel(1:3);  
            planes(planeCount).d = planeModel(4);  
            
            % 更新剩余点  
            outlierIndices = setdiff(1:size(remainingPoints,1), inlierIndices);  
            remainingPoints = remainingPoints(outlierIndices,:);  
            remainingNormals = remainingNormals(outlierIndices,:);  
        else  
            % 将不合法平面的点保留在 remainingPoints 中  
            continue;  
        end  
    else  
        continue;  
    end  

    ee=0;
end  

% % 输出统计信息  
% fprintf('平面检测完成：\n');  
% fprintf('- 总迭代次数：%d\n', iterationCount);  
% fprintf('- 检测到的平面数：%d\n', planeCount);  
% fprintf('- 剩余点数：%d\n', size(remainingPoints,1));  
% if iterationCount >= params.maxIteration  
%     fprintf('警告：达到主循环最大迭代次数 (%d)\n', params.maxIteration);  
% end  

end  




function [bestPlaneModel, bestInlierIndices] = ransacFitPlane(points, pointNormals, params)  
% ransacFitPlane 使用 RANSAC 方法拟合平面  
%  
% 输入参数：  
%   points: N×3 的点云矩阵  
%   pointNormals: N×3 的点法向量矩阵  
%   params: 参数结构体  
%  
% 输出参数：  
%   bestPlaneModel: [a, b, c, d] 平面方程参数  
%   bestInlierIndices: 内点索引  

bestPlaneModel = [];  
bestInlierIndices = [];  
bestInlierCount = 0;  
N = size(points, 1);  

% 1. 计算需要的迭代次数  
p = params.confidence;  
e = 1 - params.minPoints/N;  
s = 3; % 拟合平面需要的最小点数  
maxIters = min(params.maxIterations, ceil(log(1 - p) / log(1 - (1 - e)^s)));  

% 2. RANSAC 主循环  
for iter = 1:maxIters*10  
    % 2.1 随机选择三个点  
    idx = randperm(N, 3);  
    pt1 = points(idx(1), :);  
    pt2 = points(idx(2), :);  
    pt3 = points(idx(3), :);  
    
    % 获取对应的法向量  
    n1 = pointNormals(idx(1), :);  
    n2 = pointNormals(idx(2), :);  
    n3 = pointNormals(idx(3), :);  
    
    % 计算法向量之间的夹角  
    angleThreshold = 2; % 角度阈值（度）  
    angle12 = acosd(dot(n1, n2) / (norm(n1) * norm(n2)));  
    angle13 = acosd(dot(n1, n3) / (norm(n1) * norm(n3)));  
    angle23 = acosd(dot(n2, n3) / (norm(n2) * norm(n3)));  
    
    % 检查法向量夹角是否小于阈值  
    if abs(angle12) > angleThreshold || abs(angle13) > angleThreshold || abs(angle23) > angleThreshold  
        continue; % 如果任意两个法向量的夹角大于阈值，跳过本次采样  
    end  
    
    % 2.2 计算平面方程  
    v1 = pt2 - pt1;  
    v2 = pt3 - pt1;  
    normal = cross(v1, v2);  
    
    % 跳过共线点  
    if norm(normal) < 1e-6  
        continue;  
    end  
    
    normal = normal / norm(normal);  
    d = -dot(normal, pt1);  
    planeModel = [normal, d];  
    
    % 2.3 计算所有点到平面的距离  
    distances = abs(points * normal' + d);  
    
    % 2.4 统计内点（仅根据距离阈值）  
    inlierIndices = find(distances <= params.distThreshold);  
    inlierCount = length(inlierIndices);  
    
    % 2.5 更新最佳模型  
    if inlierCount > bestInlierCount  
        bestInlierCount = inlierCount;  
        bestPlaneModel = planeModel;  
        bestInlierIndices = inlierIndices;  
    end  
end  

% ========== 在函数的最后添加以下代码，确保内点满足角度要求 ==========  

if ~isempty(bestPlaneModel)  

    % figure
    % pcshow(points,[0.5,0.5,0.5],'MarkerSize',5); %显示点云数据
    % color1 = rand(1, 3);
    % hold on;
    % pcshow(points(bestInlierIndices, :),color1,'MarkerSize',20);

    % 提取最佳平面的法向量  
    planeNormal = bestPlaneModel(1:3); 

    
    % 确保点法向量已归一化  
    % pointNormals = normalize(pointNormals, 2);  
    pointNormals = pointNormals ./ sqrt(sum(pointNormals.^2, 2));  
    
    % 计算内点的法向量与平面法向量之间的夹角  
    inlierNormals = pointNormals(bestInlierIndices, :);    
    

    % 确保法向量方向一致（与平面法向量指向相同的半球）
    for i = 1:size(inlierNormals,1)
        if dot(inlierNormals(i,:), planeNormal) < 0
            inlierNormals(i,:) = -inlierNormals(i,:);
        end
    end

    cosTheta = inlierNormals * planeNormal';
    angles = acosd(min(max(cosTheta, -1), 1));  

   
    
    % 设置角度阈值  
    angleThresholdFinal = 10; % 目标角度阈值（度）  
    
    % 找到满足角度要求的内点  
    angleInlierIndices = bestInlierIndices(angles <= angleThresholdFinal);  
    
    % 更新内点索引  
    bestInlierIndices = angleInlierIndices;  
    bestInlierCount = length(bestInlierIndices);  
    
    % 如果没有满足条件的点，清空输出  
    if bestInlierCount < 50  
        bestPlaneModel = [];  
        bestInlierIndices = [];  
        return;  
    end  
    
    % 使用更新后的内点重新拟合平面  
    inlierPoints = points(bestInlierIndices, :);  
    centroid = mean(inlierPoints);  
    [~, ~, V] = svd(inlierPoints - centroid, 0);  
  
    normal = V(:, 3)';  
    
    % 确保平面法向量朝向与原法向量方向一致  
    if dot(normal, planeNormal) < 0  
        normal = -normal;  
    end  
    
    d = -dot(normal, centroid);  
    bestPlaneModel = [normal, d];  
end  

end

function isValid = checkPlaneValidity(planePoints, planePointNormals, planeNormal, params)  
% checkPlaneValidity 检查平面的合法性  

numPoints = size(planePoints, 1);  

% 确保法向量方向一致（与平面法向量指向相同的半球）  
for i = 1:numPoints  
    if dot(planePointNormals(i,:), planeNormal) < 0  
        planePointNormals(i,:) = -planePointNormals(i,:);  
    end  
end  

% 计算每个点的法向量与平面法向量的夹角  
% angles = zeros(numPoints, 1);  
% for i = 1:numPoints  
%     angle = acosd(dot(planePointNormals(i,:), planeNormal) / ...  
%                   (norm(planePointNormals(i,:)) * norm(planeNormal)));  
%     angles(i) = abs(angle);  % 取绝对值  
% end

cosTheta = planePointNormals * planeNormal';
angles = acosd(min(max(cosTheta, -1), 1));



% 统计不符合条件的点
invalidPoints = angles > params.angleThreshold;
invalidRatio = sum(invalidPoints) / numPoints;  

% 判断是否合法  
isValid = invalidRatio <= params.invalidRatioThreshold;  
end  



% 辅助函数：将点投影到平面  
function [Q, transform] = projectPointsToPlane1(P, planeParams)  
    % planeParams: [a,b,c,d] 表示平面 ax+by+cz+d=0  
    normal = planeParams(1:3);  
    normal = normal / norm(normal);  
    
    % 选择平面上的基向量  
    [u,v] = getPlaneBase(normal);  
    
    % 构建转换矩阵  
    transform = [u;v;normal];  

    % 计算平面原点（使用点集的质心）  
    planeOrigin = mean(P, 1);  
    
    % 投影并转换到平面坐标系  
    Q = zeros(size(P,1), 2);  
    for i = 1:size(P,1)  
        p = P(i,:);  
        % 投影到平面  
        proj = p - dot(p-planeOrigin, normal)*normal;  
        % 转换到平面坐标系  
        Q(i,:) = [dot(proj,u), dot(proj,v)];  
    end  
end  

% 辅助函数：获取平面的基向量  
function [u,v] = getPlaneBase(normal)  
    % 找到与法向量正交的两个单位向量  
    if abs(normal(3)) < abs(normal(1))  
        u = cross(normal, [0,0,1]);  
    else  
        u = cross(normal, [1,0,0]);  
    end  
    u = u / norm(u);  
    v = cross(normal, u);  
    v = v / norm(v);  
end  

function [isSymmetric,symmetryScores, axes] = checkDualAxisSymmetry(Q, tolerance)  
% Q: N×2 投影点矩阵（已转换到平面坐标系）  
% tolerance: 对称性判断的容差  
% isSymmetric: 是否双轴对称的布尔值  
% axes: 2×2 矩阵，每行表示一个对称轴的方向向量  

% 1. 计算点集的质心  
centroid = mean(Q, 1);  

% 2. 将点集平移到质心  
Q_centered = Q - centroid;  

% 3. 计算协方差矩阵  
covariance = Q_centered' * Q_centered / size(Q, 1);  

% 4. 计算特征值和特征向量  
[eigVectors, eigValues] = eig(covariance);  
eigValues = diag(eigValues);  

% 5. 将特征向量作为可能的对称轴  
axes = eigVectors';  

% 6. 对每个可能的对称轴进行验证  
symmetryScores = zeros(2, 1);  
for i = 1:2  
    axis = axes(i,:);  

    % 对每个点找到关于当前轴的对称点  
    reflectedPoints = zeros(size(Q_centered));  
    for j = 1:size(Q_centered, 1)  
        point = Q_centered(j,:);  
        % 计算点关于轴的对称点  
        proj = dot(point, axis) * axis;  
        reflectedPoints(j,:) = 2 * proj - point;  
    end  

    % 为每个点找到最近的实际点  
    minDists = zeros(size(Q_centered, 1), 1);  
    for j = 1:size(Q_centered, 1)  
        dists = sqrt(sum((Q_centered - reflectedPoints(j,:)).^2, 2));  
        minDists(j) = min(dists);  
    end  

    % 计算对称性得分  
    symmetryScores(i) = mean(minDists);  
end  

% 7. 判断是否双轴对称  
isSymmetric = all(symmetryScores < tolerance);  

% 8. 如果需要，将轴向量转换回原坐标系  
axes = axes + centroid;  
end

