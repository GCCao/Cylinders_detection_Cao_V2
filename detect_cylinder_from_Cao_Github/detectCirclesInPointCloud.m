function circles = detectCirclesInPointCloudHP(P, normalVector,minRadius,maxRadius)
% 输入:
% P: N x 3 的点云数据 (每行表示一个点的 [x, y, z] 坐标)
% normalVector: 平面的法向量 [a, b, c]，平面通过原点

% 输出:
% circles: 识别出的圆的中心和半径，格式为 M x 3 的矩阵，每行代表一个圆 [x_center, y_center, radius]

% Step 1: 将P投影到平面F上
% 确保法向量是单位向量
normalVector = normalVector / norm(normalVector);

% 计算投影矩阵
I = eye(3); % 单位矩阵
P_F = I - normalVector' * normalVector; % 投影矩阵

% 将点云投影到平面 F 上
P_proj = (P_F * P')'; % N x 3 的矩阵

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

% Step 4: 在OXY平面内采用Hough变换进行圆检测
% 忽略Z坐标，只保留X和Y坐标
P_2D = P_oxy(:, 1:2);

% 创建一个二维平面的栅格化图像，用于Hough变换
% 根据需要调整栅格的分辨率
resolution = 1; % 栅格分辨率
x_min = min(P_2D(:, 1));
x_max = max(P_2D(:, 1));
y_min = min(P_2D(:, 2));
y_max = max(P_2D(:, 2));

% 创建二维平面上的栅格
x_bins = ceil((x_max - x_min) / resolution);
y_bins = ceil((y_max - y_min) / resolution);

% 初始化二维图像矩阵
img = zeros(y_bins+20, x_bins+20); %为了不让有效像素位于图像边界，对图像进行扩展


% 将投影后的点云映射到栅格中
x_idx = ceil((P_2D(i, 1) - x_min) / resolution);
y_idx = ceil((P_2D(i, 2) - y_min) / resolution);


img(y_idx+10, x_idx+10) = img(y_idx+10, x_idx+10) + 1;
end
for i = 1:size(P_2D, 1)
    x_idx = ceil((P_2D(i, 1) - x_min) / resolution);
    y_idx = ceil((P_2D(i, 2) - y_min) / resolution);
    img(y_idx+10, x_idx+10) = img(y_idx+10, x_idx+10) + 1;
end



grayImg = rgb2gray(img); % 转换为灰度图像
img = edge(grayImg, 'canny'); % 使用Canny算法检测边缘
figure(3),imshow(img);

% 应用Hough变换检测圆
[centers, radii, metric] = imfindcircles(img, [minRadius maxRadius]);


% % 根据度量值保留五个强度最大的圆形。
% centersStrong5 = centers(1:5,:);
% radiiStrong5 = radii(1:5);
% metricStrong5 = metric(1:5);

% 在原始图像上绘制五个强度最大的圆周。

figure(4),imshow(img);
hold on
viscircles(centers, radii,'EdgeColor','b');


% 返回检测到的圆的信息
circles = [centers, radii];

end