function axesLengths = computePrincipalAxesLengths(P)  
% computePrincipalAxesLengths 计算点云的三个主轴长度  
%  
% 输入参数：  
%   P - N×3 的点云矩阵，表示 N 个三维空间中的点  
%  
% 输出参数：  
%   axesLengths - 1×3 的向量，包含三个主轴的长度  

% 1. 数据中心化（去均值）  
P_centered = bsxfun(@minus, P, mean(P));  

% 2. 进行奇异值分解（SVD）  
[~, S, ~] = svd(P_centered, 'econ');  

% 3. 提取奇异值  
singularValues = diag(S);  

% 4. 计算主轴长度（奇异值与主轴长度成正比）  
axesLengths = singularValues' * 2;  % 乘以2以获得全长度  

end