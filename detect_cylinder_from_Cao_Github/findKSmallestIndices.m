function [indices] = findKSmallestIndices(dist, K)
    % 检查K是否大于数组的长度
    if K > length(dist)
        error('K不能大于数组的长度');
    end
    
    % 使用sort函数排序，并获取排序后的索引
    [~, sortedIndices] = sort(dist);
    
    % 选择最小的K个元素的索引
    indices = sortedIndices(1:K);
end