clear all
close all

%% Load data
point = load("InliersPoints10.mat");
point = double(point.AllremainPoints);

[cylinder_model,inlier,outlier] = Least_squares_adjustment_of_Points_to_Cylindrical_Surfaces(point);


figure;
% pcshow(inlier, 'b')
pcshow(inlier,[0,1,0],'MarkerSize',50); %显示点云数据

hold on
% pcshow(outlier, 'r')
pcshow(outlier,[1,0,0],'MarkerSize',50); %显示点云数据


hold on

plot(cylinder_model)
legend('inlier point', 'outlier', 'center line of cylinder', 'cylinder model', 'TextColor', 'w')
xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');
axis equal


tic

e=0;