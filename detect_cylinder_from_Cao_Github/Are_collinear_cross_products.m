function [bool_Are_collinear,Mean_vector] = Are_collinear_cross_products(subpoints,subnormal,K,Theda_T)
%计算圆柱体上K个点的法向量之间两两叉乘的结果是否同轴.只要任意两个叉乘后的矢量夹角大于Theda_T，则不同轴，即认为这K个点不位于同一个圆柱体
%输出：（1）是否同轴；（2）如果同轴，计算平均轴向

% 计算组合的数量
numCombinations = nchoosek(K, 2);

% % 创建一个数组来存储组合结果
% combinations = zeros(1, numCombinations);

Mean_vector = [];

M = 1;
bool_Are_collinear = 1;
cross_vectors = zeros(numCombinations,3);
for i = 1:K
    for j = i+1:K
        % 存储组合到数组中
        % cross_vectors(M,:) = [i, j];
        cross_vectors(M,:) = cross(subnormal(i,:), subnormal(j,:));
        cross_vectors(M,:) = cross_vectors(M,:)/norm(cross_vectors(M,:));
        if M>=2
            dotProduct = dot(cross_vectors(M-1,:),cross_vectors(M,:));
            magnitudeA = norm(cross_vectors(M-1,:));
            magnitudeB = norm(cross_vectors(M,:));
            cosTheta = dotProduct / (magnitudeA * magnitudeB);
            cosTheta = max(min(cosTheta, 1), -1); % 限制在-1到1之间，避免数值问题
            thetaInDegrees = (acos(cosTheta))*180/3.1415926;
            %计算矢量是否同向，只要任意两个矢量夹角大于Theda_T，则认为它们不同向
            if thetaInDegrees>Theda_T
                bool_Are_collinear = 0;                
            end
        end
        M=M+1;        
    end
end


if bool_Are_collinear
    Mean_vector = mean(cross_vectors);
else
    return;
end

%判断一下平均叉乘矢量与每个叉乘矢量之间的夹角是否接近
for i = 1:(M-1)
    dotProduct = dot(Mean_vector,cross_vectors(i,:));
    magnitudeA = norm(Mean_vector);
    magnitudeB = norm(cross_vectors(i,:));
    cosTheta = dotProduct / (magnitudeA * magnitudeB);
    cosTheta = max(min(cosTheta, 1), -1); % 限制在-1到1之间，避免数值问题
    thetaInDegrees = (acos(cosTheta))*180/3.1415926;
    %计算矢量是否同向，只要任意两个矢量夹角大于Theda_T，则认为它们不同向
    if thetaInDegrees>Theda_T
        bool_Are_collinear = 0;
    end
end



end