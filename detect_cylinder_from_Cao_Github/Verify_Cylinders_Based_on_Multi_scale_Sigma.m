function VerifiedCylinders_Multi_scale_Sigma =  Verify_Cylinders_Based_on_Multi_scale_Sigma(detectedCylinders_3sigma,detectedCylinders_2sigma)
%detectedCylinders_3sigma是指在圆柱拟合是，内点是由小于3*sigma的方法判断的
%detectedCylinders_2sigma是指在圆柱拟合是，内点是由小于2*sigma的方法判断的
%原理上，detectedCylinders_2sigma比detectedCylinders_3sigma包含了更多的圆柱体，虚假圆柱也更多

%采用多尺度sigma检测的结果进行圆柱证实，其基本思路是：对于物理意义上存在的圆柱体，它们是稳定的存在，在不同倍率的sigma尺度下，
% 都能被检测出，也就是同名圆柱体对应的方向不会相差太大（只是在直径上可能有些许差异），这个观察可以在作为验证条件。
% 也就为我们提供了一条判断依据：在detectedCylinders_2sigma中任选择一个圆柱体Ci，如果在etectedCylinders_3sigma中能找到一个圆柱体Cj,
% 使得Ci和Cj轴向相近，且Ci的中心到Cj轴线的距离小于某个阈值，那么该圆柱Ci是真实圆柱的可能性极大，应该被列为真实的圆柱。



angleThreshold = 8;
distanceThreshold = 0.10; %距离之差不要超过半径的百分比

k = 0;
VerifiedCylinders_Multi_scale_Sigma = {};

for i = 1:size(detectedCylinders_2sigma,2)
    Ci = detectedCylinders_2sigma{i};
    Ci_axis_direction = Ci.Orientation;
    Ci_center = Ci.Center;
    for j = 1:size(detectedCylinders_3sigma,2)
        Cj = detectedCylinders_3sigma{j};
        Cj_axis_direction = Cj.Orientation;
        Cj_center = Cj.Center;

        %计算两个圆柱轴线的夹角（度）
        angleInDegrees = calculateAngleBetweenVectors(Ci_axis_direction, Cj_axis_direction);

        %计算圆柱Ci到Cj轴线的距离
        distance = calculate_Distance_P0_onto_Line(Cj_center, Cj_axis_direction, Ci_center);

        if angleInDegrees < angleThreshold & distance< Cj.Radius*distanceThreshold
            k = k+1;
            VerifiedCylinders_Multi_scale_Sigma{k} = Ci;
            flag = 1;
            break;
        end
    end
    
end

end

