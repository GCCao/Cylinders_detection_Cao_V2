%Efficient RANSAC for Point-Cloud Shape Detection,这这篇论文的检测结果进行测试

clear all
close all

point = load('pipe0_c.txt'); % 加载.mat文件
Cylinders = load('pipe0_c_cylinder_fitting.txt'); % 加载.mat文件


figure(1)
pcshow(point,[1,0,0],'MarkerSize',50); %显示点云数据



startp = Cylinders(:,1:3) - 300*Cylinders(:,4:6);
endp = Cylinders(:,1:3) + 300*Cylinders(:,4:6);
para = [startp, endp, Cylinders(:,7)];

for i=1:size(Cylinders,1)
    cylinder_model = cylinderModel(para(i,:));
    hold on
    plot(cylinder_model);
    ee=0;
end




rr=0;