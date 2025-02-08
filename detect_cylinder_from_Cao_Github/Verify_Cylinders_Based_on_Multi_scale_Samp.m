function VerifiedCylinders_Multi_scale_Samp =  Verify_Cylinders_Based_on_Multi_scale_Samp(detectedCylinders_MulitiSampling,angleThreshold,distanceThreshold)
%基于多尺度采样证实圆柱体真假,原理上，物理意义上存在的圆柱体，在不同采样分辨率下，都能稳定存在，而弯曲等部分的圆柱体比较随机

%采用多尺度采样检测的结果进行圆柱证实，其基本思路是：对于物理意义上存在的圆柱体，它们是稳定的存在，在不同采样率下，
% 都能被检测出，也就是同名圆柱体对应的方向不会相差太大（只是在直径上可能有些许差异），这个观察可以在作为验证条件。
% 也就为我们提供了一条判断依据：在ddetectedCylinders_MulitiSampling{1}中任选择一个圆柱体Ci，如果在ddetectedCylinders_MulitiSampling{2}和ddetectedCylinders_MulitiSampling{3}
% 中能找到一个圆柱体Cj和Ch,使得Ci与Cj和Ch的轴向都相近，且Ci的中心到Cj和Ch的轴线的距离小于某个阈值，那么该圆柱Ci是真实圆柱的可能性极大，应该被列为真实的圆柱。




k = 0;
VerifiedCylinders_Multi_scale_Samp = {};

%提取不同采样率得到采样点云的圆柱体
detectedCylinders_Samp1 = detectedCylinders_MulitiSampling{1};
detectedCylinders_Samp2 = detectedCylinders_MulitiSampling{2};
detectedCylinders_Samp3 = detectedCylinders_MulitiSampling{3};

Indexs1 = zeros(size(detectedCylinders_Samp1,2),1);
Indexs2 = zeros(size(detectedCylinders_Samp2,2),1);

for i=1:size(detectedCylinders_Samp1,2)

    Ci = detectedCylinders_Samp1{i};
    Ci_axis_direction = Ci.Orientation;
    Ci_center = Ci.Center;

    for j = 1:size(detectedCylinders_Samp2,2)

        Cj = detectedCylinders_Samp2{j};
        Cj_axis_direction = Cj.Orientation;
        Cj_center = Cj.Center;

        %计算两个圆柱轴线的夹角（度）
        angleInDegrees = calculateAngleBetweenVectors(Ci_axis_direction, Cj_axis_direction);

        %计算圆柱Ci到Cj轴线的距离
        distance = calculate_Distance_P0_onto_Line(Cj_center, Cj_axis_direction, Ci_center);

        if angleInDegrees < angleThreshold & distance< Cj.Radius*distanceThreshold
            k = k+1;
            VerifiedCylinders_Multi_scale_Samp{k} = Ci;
            Indexs1(i) = 1;
            flag = 1;
            break;
        end        

    end

end


detectedCylinders_Samp1 = VerifiedCylinders_Multi_scale_Samp;
VerifiedCylinders_Multi_scale_Samp = {};
k = 0;


for i=1:size(detectedCylinders_Samp1,2)

    Ci = detectedCylinders_Samp1{i};
    Ci_axis_direction = Ci.Orientation;
    Ci_center = Ci.Center;

    for j = 1:size(detectedCylinders_Samp3,2)

        Cj = detectedCylinders_Samp3{j};
        Cj_axis_direction = Cj.Orientation;
        Cj_center = Cj.Center;

        %计算两个圆柱轴线的夹角（度）
        angleInDegrees = calculateAngleBetweenVectors(Ci_axis_direction, Cj_axis_direction);

        %计算圆柱Ci到Cj轴线的距离
        distance = calculate_Distance_P0_onto_Line(Cj_center, Cj_axis_direction, Ci_center);

        if angleInDegrees < angleThreshold & distance< Cj.Radius*distanceThreshold
            k = k+1;
            VerifiedCylinders_Multi_scale_Samp{k} = Ci;
            flag = 1;
            break;
        end        

    end

end

end

