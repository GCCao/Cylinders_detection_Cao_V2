function [validplanes,remainingPoints] = checkPlanesValidity(planes, remainingPoints,Thstd)
% 检查平面的合法性
validplanes = struct('points',{},'normal',{},'d',{});
k = 1;
for i = 1:length(planes)

    P = planes(i).points;
    Ptemp = pointCloud(P);
    normals = surfaceNormalImpl(Ptemp, 10);

    
    [isRealPlane,angle_std] =  checkNormalConsistency(P, normals, Thstd);


    if isRealPlane
        validplanes(k) = planes(i);
        k = k+1;
    else
        remainingPoints = [remainingPoints;planes(i).points];
    end

end

% % 输出统计信息  
% fprintf('平面检测完成：\n');  
% fprintf('- 检测到的有效平面数：%d\n', k-1);  
% fprintf('- 剩余点数：%d\n', size(remainingPoints,1)); 

end