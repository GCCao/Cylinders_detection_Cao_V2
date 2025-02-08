function angle = calculateAngleBetweenLines(dirVec1, dirVec2)
    % dirVec1 和 dirVec2 是两条直线的方向向量
    % calculateAngleBetweenLines 返回两条直线之间的夹角（以度为单位）

    % 计算点积
    dotProduct = dot(dirVec1, dirVec2);
    
    % 计算两个向量的模
    normVec1 = norm(dirVec1);
    normVec2 = norm(dirVec2);
    
    % 计算夹角的余弦值
    cosTheta = dotProduct / (normVec1 * normVec2);
    
    % 确保余弦值在 [-1, 1] 范围内，以避免由于浮点数计算误差导致的问题
    cosTheta = max(min(cosTheta, 1), -1);
    
    % 计算夹角（以弧度为单位）
    thetaRadians = acos(cosTheta);
    
    % 将夹角转换为度
    angle = radtodeg(thetaRadians);
end