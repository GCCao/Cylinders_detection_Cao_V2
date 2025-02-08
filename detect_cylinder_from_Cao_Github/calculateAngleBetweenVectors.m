function angleInDegrees = calculateAngleBetweenVectors(N1, N2)
    % 计算点积
    dotProduct = dot(N1, N2);
    
    % 计算两个向量的模
    normN1 = norm(N1);
    normN2 = norm(N2);
    
    % 计算夹角的余弦值
    cosAngle = dotProduct / (normN1 * normN2);
    
    % 确保余弦值在-1到1之间，避免由于浮点数精度问题导致的计算错误
    cosAngle = max(min(cosAngle, 1), -1);
    
    % 计算角度（弧度转换为度）
    angleInRadians = acos(cosAngle);
    angleInDegrees = rad2deg(angleInRadians);
end