% RRT-connect算法
% clc
% clear
% close all
tic
global path_len pathNode_size sampleNode_size time sum_ang
%% 地图构建
map_size = [70, 50];
startPos = [3,3];
goalPos = [67,46];
map = fun_defMap; % 定义地图
stepLength = 2;            % 步长
Final = false;
%% 算法
% 定义树节点1
treeNodes1 = struct;
treeNodes1.node = startPos;
treeNodes1.parentNode = [];

% 定义树节点2
treeNodes2 = struct;
treeNodes2.node = goalPos;
treeNodes2.parentNode = [];

nodeNum1 = 1;
nodeNum2 = 1;
while Final == false
    [x_nearest,x_new,collision_flag] = getNewNode(map,map_size,treeNodes1,stepLength);
    
    % 判断父节点与子节点的连线是否跨过障碍物
    if collision_flag == 0
        continue;
    else
        % 将x_new追加到树1上
        nodeNum1 = nodeNum1 + 1;
        treeNodes1(nodeNum1).node = x_new;
        treeNodes1(nodeNum1).parentNode = x_nearest;
        
        % 计算树2到x_new最近的节点
        dist = [];
        for i = 1:nodeNum2
            dist(i) =  norm(treeNodes2(i).node - x_new);
        end
        [dist_min,idx_min] = min(dist);
        x_nearest_temp = treeNodes2(idx_min).node;
        % collision_flag = collision_check(map,x_nearest_temp,x_new);  % 判断树1的x_new与树2的x_nearest_temp连线是否满足碰撞检测
        % 
        % if dist_min < stepLength*2 && collision_flag == 1
        %     idx1_break = nodeNum1;
        %     idx2_break = idx_min;
        %     Final = true;
        %     break
        % else
            %treeNodes2(idx_min)节点朝x_new方向生长
            direction = x_new - x_nearest_temp;
            unitDirection = direction / norm(direction);
            x_new_temp = stepLength * unitDirection + x_nearest_temp;
            while true
                % 判断新生成的节点是否满足碰撞检测
                collision_flag = collision_check(map,x_nearest_temp,x_new_temp);
                if collision_flag == 1
                    nodeNum2 = nodeNum2 + 1;
                    treeNodes2(nodeNum2).node = x_new_temp;
                    treeNodes2(nodeNum2).parentNode = x_nearest_temp;
                    % 计算树1到x_new_temp最近的节点
                    dist = [];
                    for i = 1:nodeNum1
                        dist(i) =  norm(treeNodes1(i).node - x_new_temp);
                    end
                    [dist_min,idx_min_tr1] = min(dist);
                    x_nearest_temp = treeNodes1(idx_min_tr1).node;
                    % 判断树2的x_new_temp与树1的x_nearest_temp连线是否满足碰撞检测
                    collision_flag = collision_check(map,x_nearest_temp,x_new_temp);  

                    if dist_min < stepLength * 2 && collision_flag == 1
                        idx1_break = idx_min_tr1;
                        idx2_break = nodeNum2;
                        Final = true;
                        break;
                    end
                    x_nearest_temp = x_new_temp;
                    x_new_temp = x_nearest_temp + stepLength * unitDirection;
                    continue
                else
                    % 交换树节点
                    temp = treeNodes1;
                    treeNodes1 = treeNodes2;
                    treeNodes2 = temp;
                    
                    % 交换树结点数目
                    temp = nodeNum1;
                    nodeNum1 = nodeNum2;
                    nodeNum2 = temp;
                    break;
                end
            end
        % end
    end
end

%% 最优路径
path_opt = [];
idx = idx1_break;
for i = 1:2
    if i == 1
        treeNodes = treeNodes1;
    else
        path_opt = path_opt(end:-1:1,:); % 反转
        treeNodes = treeNodes2;
        idx = idx2_break;
    end
    while true
        path_opt(end+1,:) = treeNodes(idx).node;
        x_nearest = treeNodes(idx).parentNode;
        if isequal(x_nearest,startPos)
            path_opt(end+1,:) = startPos;
            break;
        elseif isequal(x_nearest,goalPos)
            path_opt(end+1,:) = goalPos;
            break;
        else
            nodes = {treeNodes.node}';
            nodes = cell2mat(nodes);
            [~, idx] = ismember(x_nearest,nodes,'rows');
        end
    end
end

% 路径长度
path_diff = diff(path_opt);
path_len = sum(sqrt(path_diff(:,1).^2 + path_diff(:,2).^2));
pathNode_size = size(path_opt, 1);
    i = 1; % 当前检测点索引
    angle = zeros(size(path_opt, 1)-2,1);
    while i <= size(path_opt, 1)-2
        % 获取连续三个路径点
        p1 = path_opt(i, :);
        p2 = path_opt(i+1, :);
        p3 = path_opt(i+2, :);
        
        % 计算路径转折角度
        vec1 = p2 - p1;   % 前向向量
        vec2 = p3 - p2;   % 后向向量
        cos_theta = dot(vec1, vec2)/(norm(vec1)*norm(vec2));
        angle(i) = acos(min(max(cos_theta,-1),1)); % 数值安全处理
        i = i + 1;
    end
sum_ang = sum(angle);
sampleNode_size = size(treeNodes2, 2) + size(treeNodes1, 2);
time = toc;

%% 画图
% plotRrtStarConnect(map_size,startPos, goalPos, map, treeNodes1,treeNodes2,path_opt)
