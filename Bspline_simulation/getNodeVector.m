% 函数：生成节点向量
function nodeVector = getNodeVector(n, k, flag)
switch flag
    case 1  % 均匀B样条
        nodeVector = linspace(0, 1, n+k+1);
        
    case 2  % 准均匀B样条
        nodeVector = zeros(1, n+k+1);
        piecewise = n - k + 2;
        for i = 1:k
            nodeVector(i) = 0;
        end
        for i = k+1:n+2
            nodeVector(i) = (i-k) / k;
            nodeVector(i) = (i-k) / piecewise;
        end
        for i = n+2:n+k+1
            nodeVector(i) = 1;
        end
        
    case 3  % 分段贝塞尔曲线
        % 先判断是否满足分段贝塞尔曲线的基本要求
        if mod(n,k-1) == 0
            nodeVector = zeros(1, n+k+1);
            
            for i = 1:k
                nodeVector(i) = 0;
            end
            for i = k+1:n+2
                nodeVector(i) = 0.5;
            end
            for i = n+2:n+k+1
                nodeVector(i) = 1;
            end
        else
            fprintf('不满足分段贝塞尔曲线要求!\n');
        end
        
    case 4  % 非均匀B样条曲线
        temp = rand(1, n+k-1);
        temp = sort(temp);
        nodeVector = [0,temp, 1];
end
