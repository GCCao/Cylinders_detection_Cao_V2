function circles = detectCirclesInPointCloudNew(P_proj, normalVector,minRadius,maxRadius)
    % 输入:
    % P_proj: N x 3 的点云数据 (每行表示一个点的 [x, y, z] 坐标)，它们是在方向为normalVector的平面上
    % normalVector: 平面的法向量 [a, b, c]，平面通过原点
    
    % 输出:
    % circles: 识别出的圆的中心和半径，格式为 M x 3 的矩阵，每行代表一个圆 [x_center, y_center, radius]
    
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


    % img =double(img);
    % magmax = max(img(:));
    % if magmax > 0
    %     img = img / magmax;
    % end
   
 
    % img = uint8(img*255);%555555555555555555555555555555555555555555555555好用
     img = uint8(img);
    % img = ;
    % figure(3),imshow(adapthisteq(img));

    % img = adapthisteq(img);

    % figure(4),imshow(img);


    % % 圆检测
    % % img = 255- img;
    % img1 = img;

    %
     imwrite(img, 'img.bmp');

    % img = imread('img1.bmp');
    % 应用Canny边缘检测
    % img = edge(img, 'canny');
   



    %  % 创建一个结构元素，用于定义膨胀和腐蚀操作的范围
    % se = strel('disk', 1); % 创建一个半径为1的圆盘形结构元素
    % 
    % % 进行膨胀操作
    % dilatedImage = imdilate(img, se);
    % 
    % % 进行腐蚀操作
    % openedImage = imerode(dilatedImage, se);
    % 
    % figure(4)
    % subplot(1, 3, 1);
    % imshow(img);
    % title('Original Image');
    % 
    % subplot(1, 3, 2);
    % imshow(dilatedImage);
    % title('Dilated Image');
    % 
    % subplot(1, 3, 3);
    % imshow(openedImage);
    % title('Opened Image');
    % 
    % img = openedImage;
    % figure(3),imshow(img);
    %

    % thresh = adaptthresh(img, 'ForegroundPolarity', 'bright', 'NeighborhoodSize', 31, 'Sensitivity', 0.6);

    thresh = median(img(:));
    % 使用计算得到的阈值进行二值化
    % img = imbinarize(img,thresh*2);
    % img = binarize(img,thresh*2);
    img = img > thresh*2;
    


    % 进行腐蚀操作
    % img = imerode(img, se);
    figure(5),imshow(img);

    % I = edge(img, 'canny');
    % imshow(I);

    % ratio_axis = 3; %长轴与短轴之比的阈值
    % ellipses = ellipse_detection_in_edgeimage(I,minRadius,maxRadius,ratio_axis);

    [centers, radii] = imfindcircles(img, [minRadius,maxRadius]);


    % 在原始图像上绘制五个强度最大的圆周。

    % figure(3),imshow(img);
    hold on
    viscircles(centers, radii,'EdgeColor','b');

    % 返回检测到的圆的信息
    circles = [centers, radii];
end