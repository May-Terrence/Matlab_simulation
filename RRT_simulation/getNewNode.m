function [x_nearest,x_new,collision_flag] = getNewNode(map,map_size,treeNodes,stepLength)
% 在地图空间随机采样撒点
x_rand(1) = map_size(1)*rand;
x_rand(2) = map_size(2)*rand;

% 依次遍历每一个树节点到采样点的距离，取最小值对应的树节点
dist = [];
for i = 1:size(treeNodes,2)
    dist(i) = norm(treeNodes(i).node - x_rand);
end
[~,idx] = min(dist);
x_nearest = treeNodes(idx).node ;

% 计算随机点方向x2的扩展步长
direction = x_rand - x_nearest;
unitDirection = direction / norm(direction);
x2 = stepLength * unitDirection;

% 生成新的节点
x_new = x_nearest + x2;

% 碰撞检测
collision_flag = collision_check(map,x_nearest,x_new);