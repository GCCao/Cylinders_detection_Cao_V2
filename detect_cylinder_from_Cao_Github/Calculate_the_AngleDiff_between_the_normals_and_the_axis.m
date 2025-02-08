function AngleDiff = Calculate_the_AngleDiff_between_the_normals_and_the_axis(model, Pre_Level_PointCloud,Pre_Level_PointCloudNormal)
%计算每个点的法向量与圆柱轴向量之间的夹角与90度的差值
 
% 初始化一个与法向量相同行数的向量以存储夹角
angles = zeros(size(Pre_Level_PointCloud, 1), 1);

% 遍历点云中每个点的法向量
for i = 1:size(Pre_Level_PointCloud, 1)
    % 计算点的法向量与模型向量的点积
    dotProduct = dot(Pre_Level_PointCloudNormal(i, :), model(4:6));
    
    % 计算夹角，使用dot积公式：cos(theta) = dot(A, B) / (||A|| * ||B||)
    cosTheta = dotProduct / (norm(Pre_Level_PointCloudNormal(i, :)) * norm(model((4:6))));
    
    % 由于点积可能由于数值问题略大于1或小于-1，使用max和min函数确保cosTheta在-1到1之间
    cosTheta = max(min(cosTheta, 1), -1);
    
    % 计算夹角的弧度值，并将结果存储在angles向量中
    angles(i) = acos(cosTheta);
end

% 将夹角从弧度转换为度，如果需要的话
angles_deg = radtodeg(angles);


AngleDiff =abs(angles_deg - 90);

end