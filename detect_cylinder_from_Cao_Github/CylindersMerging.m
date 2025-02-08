function [MergedCylinders,MergedCylinders_pointCloud] = CylindersMerging(VerifiedCylinders_Multi_scale_Samp,VerifiedCylinders_pointCloud_Multi_scale_Samp,Angular_Difference_Thre)

MergedCylinders = {};
MergedCylinders_pointCloud = {};

if size(VerifiedCylinders_Multi_scale_Samp,2)<2
    MergedCylinders = VerifiedCylinders_Multi_scale_Samp;
    MergedCylinders_pointCloud =  VerifiedCylinders_pointCloud_Multi_scale_Samp; 
    return;
end

MergedIndex = ones(size(VerifiedCylinders_Multi_scale_Samp,2),1);

for i=1:size(VerifiedCylinders_Multi_scale_Samp,2)

     Ci_Model = VerifiedCylinders_Multi_scale_Samp{i};     
    
    for j = 1:size(VerifiedCylinders_Multi_scale_Samp,2)
         Cj_Model = VerifiedCylinders_Multi_scale_Samp{j};    

        min_radius = min(Ci_Model.Radius,Cj_Model.Radius);
        max_radius = max(Ci_Model.Radius,Cj_Model.Radius);

        if i~=j && (min_radius/max_radius)>0.8 
            % Calculate the distance from point Ci_Model.Center to the axis of Cj
            p_result = ProjectPointOnLine(Cj_Model.Center,Cj_Model.Orientation,Ci_Model.Center);

            dis_proj = norm(p_result - Ci_Model.Center);

            % Calculate the distances from the projection point to the two endpoints of Cj
            d1 = norm(p_result - Cj_Model.Parameters(1:3));
            d2 = norm(p_result - Cj_Model.Parameters(4:6));

            if ( (dis_proj< Cj_Model.Radius) && (d1 +d2)< (Cj_Model.Height+0.0001) && Ci_Model.Height < Cj_Model.Height ) && abs(Cj_Model.Radius - Ci_Model.Radius) <2 % Indicate that Ci_Model.Center is located within Cj
                MergedIndex(i) = 0;
            end

        end
    end

   
end

k = 1;
for i = 1:length(MergedIndex)
    if  MergedIndex(i)>0
        MergedCylinders{k} = VerifiedCylinders_Multi_scale_Samp{i};
        MergedCylinders_pointCloud{k} = VerifiedCylinders_pointCloud_Multi_scale_Samp{i};
        k=k+1;
    end
end


end