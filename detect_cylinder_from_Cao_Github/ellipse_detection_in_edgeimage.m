function ellipses = ellipse_detection_in_edgeimage(I,minRadius,maxRadius,ratio_axis)

% % 示例使用
% clear all
% close all
% 
% % 读取边缘图像
% I = imread('image.bmp'); % 替换为实际的图像路径
% if size(I, 3) == 3
%     I = rgb2gray(I);
% end
% 
% figure
% imshow(I);
% 
% % 应用Canny边缘检测
% I = edge(I, 'canny');
% imshow(I);

% 查找所有连通轮廓
[B, L] = bwboundaries(I, 'noholes');

% 创建一个显示图像的图形窗口
figure;
imshow(I);
hold on;

% 初始化椭圆参数列表
ellipses = [];

% 遍历每个轮廓
for k = 1:length(B)
    boundary = B{k};

    % 轮廓点数足够多时进行椭圆拟合
    if length(boundary) >= 20
        % 使用fit_ellipse函数进行椭圆拟合
        ellipse_t = fit_ellipse(boundary(:,2), boundary(:,1))

        

        
        
        if ~isempty(ellipse_t)
            if ~isempty(ellipse_t.a)
               
                majorAxis = ellipse_t.a;
                minorAxis = ellipse_t.b;
                angle = ellipse_t.phi; %单位度
                centerX = ellipse_t.X0_in;
                centerY = ellipse_t.Y0_in;



                %计算椭圆度，即轮廓点到椭圆的距离
                para = [majorAxis, minorAxis, angle, centerX, centerY];
                ellipticity = calculateEllipticity(para, boundary);


                if (ellipse_t.long_axis/ellipse_t.short_axis)<ratio_axis &&  (ellipse_t.short_axis>2*minRadius) && ellipticity<0.15
                    % 绘制椭圆
                    % ellipse(majorAxis, minorAxis, deg2rad(-angle), centerX, centerY, 'r');

                    hold on 
                    plot(boundary(:,2),boundary(:,1),'b');
                    ellipse(majorAxis, minorAxis, deg2rad(-angle), centerX, centerY, 'r');

                    ellipses = [ellipses; ellipse_t];

                end
            end
        end



        ee=0;
    end
end

