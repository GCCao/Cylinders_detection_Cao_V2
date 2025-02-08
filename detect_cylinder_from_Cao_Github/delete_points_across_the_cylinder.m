%function [ValidCylinderPoints,Index_ValidCylinderPoints_inPreLevel] = delete_points_across_the_cylinder(ValidClusterPoints,Indexs_ValidClusterPoints_inPreLevel,CylinderParam)
function [bestCylinderInliersPoints,bestCylinderInliersIndex,bestCylinderModel] = delete_points_across_the_cylinder(currentClusterPoints,bestCylinderInliersIndex,bestCylinderModel)
%删除首尾不干净的点，首尾的点往往属于弯曲部分
%删除哪些不属于当前圆柱，而是属于弯曲部分或者其他圆柱上的点，并且这些点可能没有断开

CylinderParam = [bestCylinderModel.Center bestCylinderModel.Orientation bestCylinderModel.Radius];     

ValidClusterPoints = currentClusterPoints(bestCylinderInliersIndex,:);

ValidClusterPoints = double(ValidClusterPoints);


%（1）----------------将点云投影到轴线上，并且提取出起始端点
% 将所有的合法聚类点投影到轴线上
for i=1: size(ValidClusterPoints,1)
    p = ValidClusterPoints(i,:);    
    Points_projed(i,:) = ProjectPointOnLine(CylinderParam(1:3),CylinderParam(4:6),p);
end

% %可视化投影点云
% hold on
% pcshow(Points_projed,[0,1,0],'MarkerSize',50); %显示点云数据

%轴线法向量的最大分量
[maxComponent,maxComponentIndex] = max( abs(CylinderParam(4:6)') );

% 找到投影点中 maxComponentIndex
% 分量的最大值和最小值,主要是为了确保选错，比如当轴向平行于Z轴时，轴线上所有的点X坐标都一样，靠X坐标没法区分两个端点
max_CompCoord = max(Points_projed(:, maxComponentIndex));
min_CompCoord = min(Points_projed(:, maxComponentIndex));

% 找到对应最大和最小 CompCoord 值的投影点
indices_max = find(Points_projed(:, maxComponentIndex) == max_CompCoord, 1, 'first');
indices_min = find(Points_projed(:, maxComponentIndex) == min_CompCoord, 1, 'first');

% 输出两个端点
endpoint_max = Points_projed(indices_max, :);
endpoint_min = Points_projed(indices_min, :);

% %可视化两个端点
% pcshow(endpoint_min,[0,1,1],'MarkerSize',500);  
% pcshow(endpoint_max,[1,1,1],'MarkerSize',500); 


%（2）-------------------把endpoint_min和endpoint_max之间的距离等分成100份，统计每一份里面投影点的个数，相当于生成直方图
% 计算两点之间的距离，并等分
numDivisions = 100;

A = endpoint_min;
B = endpoint_max;
AB = B- A;
AB_length = norm(AB);
segment_length = AB_length / numDivisions;

unit_nromal_AB = AB/norm(AB);

% 初始化计数器数组
counts = zeros(1, numDivisions);



% 遍历每个投影点，确定它落在哪个等分区间内
for i = 1:size(Points_projed, 1)
    projected_point = Points_projed(i,:);
    
    % 计算点到A的距离
    distance_to_A = norm(A - projected_point);
    
    % 确定点落在哪个等分区间内
    segment_index = ceil(distance_to_A / segment_length);
    
    % 更新对应等分区间的计数器
    if segment_index > 0 && segment_index <= numDivisions
        counts(segment_index) = counts(segment_index) + 1;
    end
end


% figure
% X = 1:numDivisions;
% Y = counts;
% plot(X,Y);
% % (3)-----------计算点云数量之和占据95.45%(2倍sigma)的连续区域
% indices = findWithinTwoSigma(counts); %indices的第一个元素和最后一个元素，对应作新的有效区间
% 
% %找到新的起始点
% NewA = A + (indices(1)-1)*segment_length*unit_nromal_AB;
% NewB = A + indices(end)*segment_length*unit_nromal_AB;
% 
%  ee=0;


% % 
% save('counts.mat', 'counts');

%（3）-------------------对统计直方图求差分，获得拐点（或者切点）坐标
diff_P_abs = [];
for i =1:(length(counts)-3)
    diff_P_abs(i) = abs(counts(i+3) - counts(i));
end

% 找到左侧梯度绝对值最大的点
half_legth = round(length(diff_P_abs)/2);
[max_peak_left, max_peak_left_index]   = max(diff_P_abs(1 : half_legth));
[max_peak_right, max_peak_right_index] = max(diff_P_abs((half_legth+1) : length(diff_P_abs)));

max_peak_left_index  = max_peak_left_index ;
max_peak_right_index =  half_legth + max_peak_right_index ;




[meanvalue] = mean(counts') ;

%如果差分后的峰值处统计点数量比均值低，说明这一侧不是合理的圆柱部分
bool_delete_left = 0;    
if counts(max_peak_left_index)<meanvalue
    bool_delete_left = 1;    
end

bool_delete_right = 0;    
if counts(max_peak_right_index+3)<meanvalue
    bool_delete_right = 1;    
end


% % 为了让圆柱段更加纯粹，对两个关键位置点进行向内收缩1个单位
max_peak_left_index  =  max_peak_left_index + 2;
max_peak_right_index =  max_peak_right_index -1;



% hold on 
% plot(max_peak_left_index,counts(max_peak_left_index),'ro');
% plot(max_peak_right_index,counts(max_peak_right_index),'bo');


C = []; %valid_start_point
D = []; %valid_end_point
if bool_delete_left ==1 && bool_delete_right ==0
    C = A + (max_peak_left_index*segment_length)*unit_nromal_AB;
    D = B;
elseif bool_delete_left ==0 && bool_delete_right ==1
    C = A ;
    D = A + (max_peak_right_index*segment_length)*unit_nromal_AB;
    % hold on
    % pcshow(valid_end_point,[0,0,1],'MarkerSize',500);
elseif bool_delete_left ==1 && bool_delete_right ==1
    C = A + (max_peak_left_index*segment_length)*unit_nromal_AB;
    D = A + (max_peak_right_index*segment_length)*unit_nromal_AB;
else
    C = A;
    D = B;
end

% save('ValidClusterPoints.mat', 'ValidClusterPoints');

% 
% figure
% hold on
% pcshow(ValidClusterPoints,[1,0,0],'MarkerSize',50);
% hold on
% pcshow(C,[0,1,0],'MarkerSize',500);
% hold on
% pcshow(D,[1,1,0],'MarkerSize',500);


%（4）------------找到位于C和D之间的投影点，以及对应的圆柱点
Index_ValidPoint = zeros(size(Points_projed,1),1);
%凡是位于和valid_end_point的投影点，都认为是来自圆柱，找到他们的索引
for i = 1:size(Points_projed, 1)
    projected_point = Points_projed(i,:);

    % 计算点到valid_start_point的距离
    distance_to_valid_start_point = norm(C - projected_point);

    % 计算点到valid_end_point的距离
    distance_to_valid_end_point   = norm(D - projected_point);

    % 计算点到valid_start_point和valid_end_point的距离
    distance_to_2points   = norm(D - C);

    %如果某个点到两个端点的距离之和等于两个端点的距离，说明该点位于这两个点之间
    if (distance_to_valid_start_point + distance_to_valid_end_point)  <  (distance_to_2points + 0.0001)
        Index_ValidPoint(i,1) = 1;
    end      
end

%找到纯粹的圆柱点索引及坐标
Index_ValidCylinderPoints_incurrent = find(Index_ValidPoint(:,1) == 1);

bestCylinderInliersIndex = Index_ValidCylinderPoints_incurrent;

bestCylinderInliersPoints = ValidClusterPoints(Index_ValidPoint(:,1) == 1,:);

% hold on
% pcshow(ValidCylinderPoints,[0,1,0],'MarkerSize',500);


%对圆柱的参数进行更新，这里保持法向不变
% bestCylinderModel.Parameters(1:6) = [C D];
% bestCylinderModel.Center = (C+D)/2;
% bestCylinderModel.Height = norm(C-D);

%对圆柱的参数进行更新，这里保持法向和半径不变
R = bestCylinderModel.Radius;
para = [C, D, R];
bestCylinderModel = cylinderModel(para); %注意：para由首尾两点坐标和半径组成

end