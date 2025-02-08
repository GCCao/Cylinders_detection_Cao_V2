function Draw_arrows_and_labels_at_the_centerpoint_of_the_cylinder(bestCylinderModel,number,Sampling_Order)

%%%%%%%%%%%%%%%%%%%%%%%%%%%绘制圆柱体的中心点以及法向----start
centerstartP = bestCylinderModel.Center;
axisDirection = bestCylinderModel.Orientation/norm(bestCylinderModel.Orientation);
% 找到与轴线正交的向量，这里我们选择与轴线的叉积结果作为正交向量
orthogonalLline_direction = cross(axisDirection, [1, 0, 0]); % 选择与axisDirection和x轴的叉积
% 确保L_direction不是零向量
if norm(orthogonalLline_direction) < 0.0001
    orthogonalLline_direction = cross(axisDirection, [0, 1, 0]); % 如果与x轴的叉积为零，则选择与y轴的叉积
end
orthogonalLline_direction = orthogonalLline_direction/norm(orthogonalLline_direction);
zoomfactor= 3*bestCylinderModel.Radius;

figure(Sampling_Order)
hold on
quiver3(centerstartP(1), centerstartP(2), centerstartP(3), ...
    orthogonalLline_direction(1)*zoomfactor, orthogonalLline_direction(2)*zoomfactor, orthogonalLline_direction(3)*zoomfactor, ...
    'MaxHeadSize', 5,'Color', 'magenta', 'LineWidth', 5); %MaxHeadSize表示箭头大小， 'r'表示红色箭头，'LineWidth'设置线宽，

%%%%%%%%%%%%%%%%%%%%%%%%%%%绘制圆柱体的中心点以及法向----end

% % 在箭头起始点添加数字

text(centerstartP(1)+orthogonalLline_direction(1)*zoomfactor, centerstartP(2)+orthogonalLline_direction(2)*zoomfactor, centerstartP(3)+orthogonalLline_direction(3)*zoomfactor, num2str(number), 'FontSize', 14, 'Color', 'green');
end