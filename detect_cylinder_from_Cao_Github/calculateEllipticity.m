function ellipticity = calculateEllipticity(para, P)
    % para = [majorAxis, minorAxis, angle, centerX, centerY]
    % P is a Nx2 matrix representing the contour points

    % 解构椭圆参数
    majorAxis = para(1);
    minorAxis = para(2);
     angle = deg2rad(-para(3)); % 将角度转换为弧度
    % angle = -para(3)*3.1415926/180; % 将角度转换为弧度
    
    centerX = para(4);
    centerY = para(5);




    % 计算旋转矩阵，用于将点旋转到椭圆的标准位置
    R = [cos(angle), -sin(angle); sin(angle), cos(angle)];

    % 初始化总距离
    totalDistance = 0;


    % 遍历每个轮廓点
    for i = 1:size(P, 1)
        % 将点相对于椭圆中心平移
       % point = P(i, :) - [centerX, centerY];
        point = P(i, :) - [centerY, centerX]; %在matlab中要注意图像中x和y的关系%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % 将点旋转以匹配椭圆的方向
        rotatedPoint = (R * point')';

        % 获取旋转后的坐标
        x = rotatedPoint(1);
        y = rotatedPoint(2);

        % hold on
        % 
        % plot(y,x, 'go', 'MarkerSize', 2, 'MarkerFaceColor', 'g');
        % axis equal;



        % 计算点到椭圆的距离
        distance = sqrt((x^2 / majorAxis^2) + (y^2 / minorAxis^2)) - 1;

        % 累加距离的绝对值
        totalDistance = totalDistance + abs(distance);
    end

    % 计算平均距离（椭圆度）
    ellipticity = totalDistance / size(P, 1);
end