function [WingPoints,WingPointsIndex] = Find_Wing_points_Index(expandedCylinderInliersIndex,currentClusterPoints,A,B,arc_length_ratio_proj)

if norm(A-B)<0.0001
    WingPoints = [];
    WingPointsIndex = [];
    return;
end

%投影弧长占比的阈值，低于这个阈值，认为这部分点云大概率不属于圆柱
% arc_length_ratio_proj = 0.25;

P = currentClusterPoints(expandedCylinderInliersIndex,:);

%计算P在AB直线上的投影，注意：P_proj中最远的两个点未必就是A和B,因为这里的A和B来自圆柱体模型的端点，而P是采用额外的方法获得圆柱体表面点
P_proj = projectPointsToLine_by_MatrixOperation(P, A, B);

if size(P_proj,2)<3
    WingPoints = [];
    WingPointsIndex = [];
    return;
end
%求P_proj中最远的两个点。如果直接计算两两的距离，非常耗时，因为P_proj点数可能很大
[C, D, maxDistance] = findFarthestPoints_inP_proj_new(P_proj);


%把P分割成100段
% segments = segmentPointCloud(P, C, D);
if norm(C-D)<0.0001
    WingPoints = [];
    WingPointsIndex = [];
    return;
end

[segments,segments_index] = segmentPointCloud_new8(P, C, D);  

%计算点云数量最大的分段及数量值
maxnum = 0;
for j=1:length(segments)
    PPP = segments{j};
    n  = size(PPP,1);
    if n>maxnum
        maxnum = n;
    end
end


%小圆柱上至少应该具备的点云数量
min_numpoints = maxnum*arc_length_ratio_proj;

%找到不满足点数要求的小圆柱,将它们当做翅膀点云
WingPoints = [];
WingPointsIndex = [];
for j=1:length(segments)
    PP = segments{j};
    II = segments_index{j};
    n  = size(PP,1);
    if n<min_numpoints
        WingPoints = [WingPoints;PP];
        WingPointsIndex = [WingPointsIndex;II];
    end
end



end