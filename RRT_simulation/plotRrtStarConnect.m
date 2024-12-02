function plotRrtStarConnect(map_size,startPos, goalPos, map, treeNodes1,treeNodes2,path_opt)
figure
%% 可视化地图
% 画起点和终点
scatter(startPos(1),startPos(2),80,'g','filled','s')
hold on
scatter(goalPos(1),goalPos(2),80,'r','filled','h')

% 画边界障碍物
for i = 1:size(map.obs_boundary,2)
    vertexs = map.obs_boundary{i}{1};
    fill(vertexs(:,1),vertexs(:,2),'k'); hold on;
end
% 画多边形障碍物
for i = 1:size(map.obs_polygon,2)
    % per obs
    vertexs = map.obs_polygon{i}{1};
    fill(vertexs(:,1),vertexs(:,2),'k'); hold on;
end
% 画圆形障碍物
t = linspace(0, 2*pi);
for i = 1:length(map.obs_circle)
    x = cos(t)*map.obs_circle(i,3)+map.obs_circle(i,1);
    y = sin(t)*map.obs_circle(i,3)+map.obs_circle(i,2);
    fill(x,y,'k'); hold on;
end
axis equal
xlim([0 map_size(1)+1]);
ylim([0 map_size(2)+1]);

%% 画采样点与路径
for i = 2:size(treeNodes1,2)
    plot([treeNodes1(i).parentNode(1),treeNodes1(i).node(1)],...
        [treeNodes1(i).parentNode(2),treeNodes1(i).node(2)],'b','linewidth',1);
%     pause(0.01)
end
for i = 2:size(treeNodes2,2)
    plot([treeNodes2(i).parentNode(1),treeNodes2(i).node(1)],...
        [treeNodes2(i).parentNode(2),treeNodes2(i).node(2)],'c','linewidth',1);
%         pause(0.01)

end
plot(path_opt(:,1),path_opt(:,2),'r--','linewidth',1)

end