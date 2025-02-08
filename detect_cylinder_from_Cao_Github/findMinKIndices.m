function [indices] = findMinKIndices(dist, K)
%找到最小的K个元素，并返回索引

% 检查输入参数是否有效
if nargin ~= 2
    error('需要两个输入参数：dist数组和整数K');
end

if ~isnumeric(dist) || ~isvector(dist)
    error('dist必须是数值型向量');
end

if ~isnumeric(K) || ~isscalar(K) || K <= 0 || K > length(dist)
    error('K必须是正整数，且不大于dist的长度');
end

% 对dist进行排序，返回排序后的数组
sortedDist = sort(dist);

% 找到排序后数组中最小的K个元素的索引
% 注意，sortedDist是升序排列的，因此我们从尾部开始取K个元素
indices = find(sortedDist(end-K+1:end) == dist, 1);

% 如果有多个相同的最小值，find函数可能返回多个索引
% 这里我们只返回第一个出现的索引
if isempty(indices)
    indices = find(dist == sortedDist(end-K+1), 1);
end
end