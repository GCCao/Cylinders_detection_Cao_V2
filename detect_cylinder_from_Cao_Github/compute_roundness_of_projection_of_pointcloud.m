 function roundness = compute_roundness_of_projection_of_pointcloud(P,planePara,cylinder_model)
%将点云向一个平面投影，投影之后，在图像中检测点云形成的轮廓，并计算轮廓的圆度，如果圆度满足要求，则认为原点云可能是圆柱


Points_temp = pointCloud(P);
if isempty(Points_temp.Normal)
    % Use 6 neighboring points to estimate a normal vector
    PNormal = surfaceNormalImpl(Points_temp, 6);
end


% planePara = [-0.7156, -0.6984, 0.0161]; %ptCloudMov = pcread("BigTube_AllScanedPC_local.ply");

% 平面法向量
plane_normal = planePara(1:3);
plane_normal = plane_normal / norm(plane_normal); % 归一化

% Step1 计算点云的投影点
[P_proj,P_F,valid_indices] = project_point_cloud_into_plane(P, PNormal, plane_normal(1), plane_normal(2), plane_normal(3),10);
% valid_indices： 夹角与90度之差小于10度的点的索引
% P_F:投影矩阵

%将圆柱模型轴线上一点投影到平面上
%P_prj = project_Point_onto_plane(P, a,b,c,d)
circlecenter_prj = project_Point_onto_plane(cylinder_model.Center, plane_normal(1), plane_normal(2), plane_normal(3),0);

%Step2 将投影点转换为图像,同时获得圆柱模型在图像中对应的圆心和半径（单位:像素）
[img,circle_InImgage] = ProjPoint2Image(P_proj, plane_normal,circlecenter_prj,cylinder_model.Radius);


% figure,imshow(img);
% hold on
% viscircles([circle_InImgage(1),circle_InImgage(2)], circle_InImgage(3),'EdgeColor','b');


%%----------计算图像中投影点到圆的距离及方差
% 获取灰度为0的点的索引
[rows, cols] = find(img > 0); % 获取灰度为0的点的坐标

% 初始化距离数组
distances = zeros(length(rows), 1);

% 计算每个灰度为0的点到圆C轮廓的距离
for i = 1:length(rows)
    % 计算点到圆心的欧几里得距离
    d = sqrt((cols(i) - circle_InImgage(1))^2 + (rows(i) - circle_InImgage(2))^2);
    
    % 计算到圆C轮廓的距离
    distances(i) = abs(d - circle_InImgage(3));
end

% 计算距离的方差 
distancevar = var(distances);

maxd = max(distances(:));
mind = min(distances(:));
stepd = (maxd-mind)/3;

if length(circle_InImgage)<3
    roundness = Inf;
    return;
end

%圆度定义为方差除以半径
roundness = distancevar/circle_InImgage(3)



% 指定直方图的区间边界
binEdges = [mind, mind+stepd, mind+2*stepd, maxd];

% % 计算直方图
% figure; % 创建一个新的图形窗口
% histogram(distances, 'BinEdges', binEdges);



 end