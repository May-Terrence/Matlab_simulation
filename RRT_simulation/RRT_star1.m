% RRT_star算法
% clc
% clear
close all
tic
global len pathNode sampleNode time 
%% 地图构建
map_size = [50, 30];
startPos = [2,2];
goalPos = [49,26];
map = fun_defMap;
stepLength = 2;            % 步长
rou = 5;                   % 圆形区域半径

%% 算法
% 定义树节点，第一列放节点编号，第二列放该节点的父节点
treeNodes1 = struct;
treeNodes1.node = startPos;
treeNodes1.parentNode = [];
treeNodes1.dist = 0;

nodeNum = 1;
while true
    % 生成随机点
    x_rand(1) = map_size(1)*rand;
    x_rand(2) = map_size(2)*rand;
    
    % 依次遍历每一个树节点到采样点的距离，取最小值对应的树节点
    dist = [];
    for i = 1:size(treeNodes1,2)
        dist(i) = norm(treeNodes1(i).node - x_rand);
    end
    [~,idx_min] = min(dist);
    x_nearest = treeNodes1(idx_min).node;
    
    % 计算随机点方向x2的扩展步长
    direction = x_rand - x_nearest;
    unitDirection = direction / norm(direction);
    x2 = stepLength * unitDirection;
    
    % 生成新的节点
    x_new = x_nearest + x2;
        
    % 碰撞检测
    collision_flag = collision_check(map,x_nearest,x_new);
    
    % 判断父节点与子节点的连线是否跨过障碍物
    if collision_flag == 0
        continue
    else
        % 追加节点
        nodeNum = nodeNum + 1;
        treeNodes1(nodeNum).node = x_new;
        treeNodes1(nodeNum).parentNode = x_nearest;
        treeNodes1(nodeNum).dist = treeNodes1(idx_min).dist + stepLength;
            
        % 依次遍历除了x_new之外的每一个树节点到x_new的距离，选取位于圆形区域内的树节点
        idx_nearest = [];
        for i = 1:size(treeNodes1,2)-1
            collision_flag = collision_check(map,treeNodes1(i).node,x_new);
            dist = norm(treeNodes1(i).node - x_new);
            if dist < rou && collision_flag == 1
                idx_nearest(end+1,:) = i;
            end
        end
        
        % 判断圆形区域内哪一个节点到源节点的距离更近
        if isempty(idx_nearest)
            continue
        else
            dist = [];
            for i = 1:length(idx_nearest)
                idx = idx_nearest(i);
                dist(i) = norm(treeNodes1(idx).node - x_new) + treeNodes1(idx_nearest(i)).dist;
            end
            [dist_min,idx_min] = min(dist);
            treeNodes1(nodeNum).parentNode = treeNodes1(idx_nearest(idx_min)).node;
            treeNodes1(nodeNum).dist = dist_min;
            
            % 更新圆内的其他最近邻节点，判断经过x_new到达此邻接点路径是否更优
            for i = 1:length(idx_nearest)
                idx = idx_nearest(i);
                collision_flag = collision_check(map,treeNodes1(idx).node,x_new);
                dist_temp = norm(treeNodes1(idx).node - x_new);
                if collision_flag == 1 && dist_min + dist_temp < treeNodes1(idx).dist
                    treeNodes1(idx).parentNode = x_new;
                    treeNodes1(idx).dist = dist_min + dist_temp;
                end
            end
        end
    end    
    
    % 判断子节点是否位于目标区域
    if norm(treeNodes1(end).node - goalPos) < stepLength
        break;
    end
end

%% 最优路径
path_opt = goalPos;
idx = size(treeNodes1,2);
while true
    path_opt(end+1,:) = treeNodes1(idx).node;
    x_nearest = treeNodes1(idx).parentNode;
    if isequal(x_nearest,startPos)
        path_opt(end+1,:) = startPos;
        break;
    else
        nodes = {treeNodes1.node}';
        nodes = cell2mat(nodes);
        [~, idx] = ismember(x_nearest,nodes,'rows');
    end
end

% 路径长度
path_diff = diff(path_opt);
len = sum(sqrt(path_diff(:,1).^2 + path_diff(:,2).^2));
pathNode = size(path_opt, 1);
sampleNode = size(treeNodes1, 2);
time = toc;

%% 画图
% plotFigureRrtStar(map_size,startPos, goalPos, map, treeNodes1,path_opt)