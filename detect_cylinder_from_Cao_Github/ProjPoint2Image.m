% function img = ProjPoint2Image(P_proj, plane_normal,cylinder_model)
function [img,circle_InImgage] = ProjPoint2Image(P_proj, normalVector,circlecenter_prj,Radius)
% 将空间平面内的点，转换成图像，也就是栅格化.
% 并将圆柱投影到图像中后，其对应的图像半径和以及图像圆心

% Step 1: 确保法向量是单位向量
normalVector = normalVector / norm(normalVector);


% Step 2: 计算F平面到OXY平面的刚性变换矩阵H
% 构造旋转矩阵，使得法向量 normalVector 映射到 z 轴 [0, 0, 1]
z_axis = [0, 0, 1]';
rotationAxis = cross(normalVector, z_axis);
rotationAxis = rotationAxis / norm(rotationAxis);
angle = acos(dot(normalVector, z_axis));

% 使用Rodrigues公式计算旋转矩阵
K = [0 -rotationAxis(3) rotationAxis(2);
    rotationAxis(3) 0 -rotationAxis(1);
    -rotationAxis(2) rotationAxis(1) 0];
R = eye(3) + sin(angle) * K + (1 - cos(angle)) * (K * K);

% Step 3: 根据H将P_proj中的点变换到OXY平面，得到新的点集P_oxy
P_oxy = (R * P_proj')'; % N x 3 的矩阵

% Step 4: 在OXY平面内进行圆检测
% 忽略Z坐标，只保留X和Y坐标
P_2D = P_oxy(:, 1:2);

% 创建一个二维平面的栅格化图像
% 根据需要调整栅格的分辨率
resolution = 0.5; % 栅格分辨率
x_min = min(P_2D(:, 1));
x_max = max(P_2D(:, 1));
y_min = min(P_2D(:, 2));
y_max = max(P_2D(:, 2));

% 创建二维平面上的栅格
x_bins = ceil((x_max - x_min) / resolution);
y_bins = ceil((y_max - y_min) / resolution);

% 初始化二维图像矩阵
img = zeros(y_bins+100, x_bins+100);

% 将投影后的点云映射到栅格中
for i = 1:size(P_2D, 1)
    x_idx = ceil((P_2D(i, 1) - x_min) / resolution);
    y_idx = ceil((P_2D(i, 2) - y_min) / resolution);
    img(y_idx+50, x_idx+50) = img(y_idx+50, x_idx+50) + 1;
end

% img = uint8(img*255);%555555555555555555555555555555555555555555555555好用
img = uint8(img);

% img =double(img);
% magmax = max(img(:));
% if magmax > 0
%     img = img / magmax;
% end

thresh = median(img(:));
img = img > thresh*2;






%%%%%%%%%%%%单独计算圆柱中心在图像中的像素坐标，以及圆柱在图像中的半径。代替图像中的圆检测
% 根据H将circlecenter_prj点变换到OXY平面，得到新的点P_oxy_center
P_oxy_center = (R * circlecenter_prj')'; % N x 3 的矩阵

% 忽略Z坐标，只保留X和Y坐标
P_2D_center = P_oxy_center(:, 1:2);

% 将投影后的点映射到栅格中
x_idx_center = ceil((P_2D_center(1) - x_min) / resolution) +50;
y_idx_center = ceil((P_2D_center(2) - y_min) / resolution) +50;

%存储图像中圆的圆心坐标和半径
circle_InImgage = [x_idx_center,y_idx_center,Radius/resolution];

end