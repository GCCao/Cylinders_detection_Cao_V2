function Mdl = KNN_FeatureVectors_Model(P)
% 加载数据
% 假设P是Nxcolsize矩阵,第colsize列是标签
colsize = size(P,2);

% 分离特征向量和标签
XTrain = P(:, 1:(colsize-1)); % 特征向量
yTrain = P(:, colsize); % 标签

% % 分割数据集为训练集和测试集
% cv = cvpartition(size(X, 1), 'HoldOut', 0.3);
% idx = cv.test;

% % 创建训练集和测试集
% XTrain = X(~idx, :);
% yTrain = y(~idx, :);
% XTest = X(idx, :);
% yTest = y(idx, :);

% 训练KNN模型
Mdl = fitcknn(XTrain, yTrain, 'NumNeighbors', 5);

end