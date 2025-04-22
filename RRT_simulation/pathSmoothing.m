function smoothed_path = pathSmoothing(path_opt,map)
% PATHSMOOTHING 路径角度约束平滑优化函数
% 输入参数：
%   original_path : n×2矩阵，原始路径坐标点集
%   theta         : 角度阈值(弧度)，小于该值则尝试优化
%   collision_check : 函数句柄，判断两点间直线是否碰撞
% 输出参数：
%   smoothed_path : 优化后的n×2路径点集
% figure;
% plot(path_opt(:,1),path_opt(:,2));
    smoothed_path = path_opt; % 初始化起点
    i = 1; % 当前检测点索引
    
    while i <= size(smoothed_path, 1)-2
        % 获取连续三个路径点
        p1 = smoothed_path(i, :);
        p2 = smoothed_path(i+1, :);
        p3 = smoothed_path(i+2, :);
        
        % 计算路径转折角度
        % vec1 = p1 - p2;   % 前向向量
        % vec2 = p3 - p2;   % 后向向量
        % cos_theta = dot(vec1, vec2)/(norm(vec1)*norm(vec2));
        % angle = acos(min(max(cos_theta,-1),1)); % 数值安全处理
        
        % if angle < pi*(180/180)
            % 尝试直连首尾点
            if collision_check(map,p1,p3) == 1
                % 删除中间点，直接连接p1-p3
                smoothed_path(i+1,:) = [];
            else
                % 碰撞情况下尝试次优化：保留最近可连接点
                % mid_point = (p1 + p3)/2;
                % if ~collision_check(p1, mid_point)
                %     smoothed_path(i+1,:) = mid_point;
                % end
                i = i + 1;
            end
        % else
        %     % 角度满足要求，移至下一节点
        %     i = i + 1;
        % end
    end

end