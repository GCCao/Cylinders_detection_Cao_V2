function [FinalCylinders, FinalCylinders_pointCloud] = Delete_short_cylinders(MergedCylinders,MergedCylinders_pointCloud)

FinalCylinders = {};
FinalCylinders_pointCloud = {};

if size(MergedCylinders,2)<2
    FinalCylinders = MergedCylinders;
    FinalCylinders_pointCloud =  MergedCylinders_pointCloud; 
    return;
end

MergedIndex = ones(size(MergedCylinders,2),1);

for i=1:size(MergedCylinders,2)

     Ci_Model = MergedCylinders{i};   

     if Ci_Model.Height > 2*Ci_Model.Radius
         continue;
     end
    
     %找到距离当前圆柱中心最近的圆柱中心
     dmax = 100000000;
     for j = 1:size(MergedCylinders,2)
         Cj_Model = MergedCylinders{j};
         if i~=j
             d = norm(Ci_Model.Center - Cj_Model.Center);
             if d<dmax
                 min_j = j;
                 dmax = d;
             end
         end
     end

     %计算当前圆柱与最近圆柱的轴线夹角
     Cj_Model = MergedCylinders{min_j}; 
     angleInDegrees = calculateAngleBetweenVectors(Ci_Model.Orientation, Cj_Model.Orientation);
      
     if angleInDegrees >= 90
         angleInDegrees = 180-angleInDegrees;
     elseif angleInDegrees < 90 && angleInDegrees >= 45
         angleInDegrees = 90-angleInDegrees;
     end

     if angleInDegrees > 3  
         MergedIndex(i) = 0;
     end
   
end

k = 1;
for i = 1:length(MergedIndex)
    if  MergedIndex(i)>0
        FinalCylinders{k} = MergedCylinders{i};
        FinalCylinders_pointCloud{k} = MergedCylinders_pointCloud{i};
        k=k+1;
    end
end


end