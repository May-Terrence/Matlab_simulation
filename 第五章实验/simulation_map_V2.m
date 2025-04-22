% function plotRrtStarConnect(map_size,startPos, goalPos, map, treeNodes1,treeNodes2,path_opt)
load('第五章实验\仿真实验地图_V2.mat');
RRTC = figure;
%% 可视化地图
% 画起点和终点
ax1 = subplot(1, 2, 1);
scatter(startPos(1),startPos(2),80,'b','filled','pentagram')
hold on
scatter(goalPos(1),goalPos(2),80,'g','filled','pentagram')
title(ax1,'\fontname{宋体}低密度障碍物地图');
% 画多边形障碍物
for i = 1:size(map.obs_polygon_boom,2)
    % per obs
    vertexs = map.obs_polygon_boom{i}{1};
    fill(vertexs(:,1),vertexs(:,2), [0.7 0.7 0.7],'EdgeColor', 'none','HandleVisibility', 'off'); hold on;
end

for i = 1:size(map.obs_polygon,2)-1
    % per obs
    vertexs = map.obs_polygon{i}{1};
    fill(vertexs(:,1),vertexs(:,2), [0.3 0.3 0.3],'EdgeColor', 'none', 'HandleVisibility', 'off'); hold on;
end
    vertexs = map.obs_polygon{size(map.obs_polygon,2)}{1};
    fill(vertexs(:,1),vertexs(:,2), [0.3 0.3 0.3],'EdgeColor', 'none'); hold on;

% 画圆形障碍物
t = linspace(0, 2*pi);
for i = 1:length(map.obs_circle_boom)-1
    x = cos(t)*map.obs_circle_boom(i,3)+map.obs_circle_boom(i,1);
    y = sin(t)*map.obs_circle_boom(i,3)+map.obs_circle_boom(i,2);
    fill(x,y,[0.7 0.7 0.7],'EdgeColor', 'none','HandleVisibility', 'off'); hold on;
end
    x = cos(t)*map.obs_circle_boom(length(map.obs_circle_boom),3)+map.obs_circle_boom(length(map.obs_circle_boom),1);
    y = sin(t)*map.obs_circle_boom(length(map.obs_circle_boom),3)+map.obs_circle_boom(length(map.obs_circle_boom),2);
    fill(x,y,[0.7 0.7 0.7],'EdgeColor', 'none'); hold on;

t = linspace(0, 2*pi);
for i = 1:length(map.obs_circle)-1
    x = cos(t)*map.obs_circle(i,3)+map.obs_circle(i,1);
    y = sin(t)*map.obs_circle(i,3)+map.obs_circle(i,2);
    fill(x,y,[0.3 0.3 0.3],'EdgeColor', 'none','HandleVisibility', 'off'); hold on;
end
    x = cos(t)*map.obs_circle(length(map.obs_circle),3)+map.obs_circle(length(map.obs_circle),1);
    y = sin(t)*map.obs_circle(length(map.obs_circle),3)+map.obs_circle(length(map.obs_circle),2);
    fill(x,y,[0.3 0.3 0.3],'EdgeColor', 'none'); hold on;

axis equal
xlim([0 map_size(1)]);
ylim([0 map_size(2)]);

ylabel('\fontname{宋体}距离\fontname{Times New Roman}(m)');
xlabel('\fontname{宋体}距离\fontname{Times New Roman}(m)');
set(RRTC.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', ...
    1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')

%%
ax2 = subplot(1, 2, 2);
scatter(startPos(1),startPos(2),80,'b','filled','pentagram')
hold on
scatter(goalPos(1),goalPos(2),80,'g','filled','pentagram')
title(ax2,'\fontname{宋体}高密度障碍物地图');
% 画多边形障碍物
for i = 1:size(map1.obs_polygon_boom,2)
    % per obs
    vertexs = map1.obs_polygon_boom{i}{1};
    fill(vertexs(:,1),vertexs(:,2), [0.7 0.7 0.7],'EdgeColor', 'none','HandleVisibility', 'off'); hold on;
end

for i = 1:size(map1.obs_polygon,2)-1
    % per obs
    vertexs = map1.obs_polygon{i}{1};
    fill(vertexs(:,1),vertexs(:,2), [0.3 0.3 0.3],'EdgeColor', 'none', 'HandleVisibility', 'off'); hold on;
end
    vertexs = map1.obs_polygon{size(map1.obs_polygon,2)}{1};
    fill(vertexs(:,1),vertexs(:,2), [0.3 0.3 0.3],'EdgeColor', 'none'); hold on;

% 画圆形障碍物
t = linspace(0, 2*pi);
for i = 1:length(map1.obs_circle_boom)-1
    x = cos(t)*map1.obs_circle_boom(i,3)+map1.obs_circle_boom(i,1);
    y = sin(t)*map1.obs_circle_boom(i,3)+map1.obs_circle_boom(i,2);
    fill(x,y,[0.7 0.7 0.7],'EdgeColor', 'none','HandleVisibility', 'off'); hold on;
end
    x = cos(t)*map1.obs_circle_boom(length(map1.obs_circle_boom),3)+map1.obs_circle_boom(length(map1.obs_circle_boom),1);
    y = sin(t)*map1.obs_circle_boom(length(map1.obs_circle_boom),3)+map1.obs_circle_boom(length(map1.obs_circle_boom),2);
    fill(x,y,[0.7 0.7 0.7],'EdgeColor', 'none'); hold on;

t = linspace(0, 2*pi);
for i = 1:length(map1.obs_circle)-1
    x = cos(t)*map1.obs_circle(i,3)+map1.obs_circle(i,1);
    y = sin(t)*map1.obs_circle(i,3)+map1.obs_circle(i,2);
    fill(x,y,[0.3 0.3 0.3],'EdgeColor', 'none','HandleVisibility', 'off'); hold on;
end
    x = cos(t)*map1.obs_circle(length(map1.obs_circle),3)+map1.obs_circle(length(map1.obs_circle),1);
    y = sin(t)*map1.obs_circle(length(map1.obs_circle),3)+map1.obs_circle(length(map1.obs_circle),2);
    fill(x,y,[0.3 0.3 0.3],'EdgeColor', 'none'); hold on;

axis equal
xlim([0 map_size(1)]);
ylim([0 map_size(2)]);

ylabel('\fontname{宋体}距离\fontname{Times New Roman}(m)');
xlabel('\fontname{宋体}距离\fontname{Times New Roman}(m)');
h = legend('\fontname{宋体}起点', '\fontname{宋体}目标点','\fontname{宋体}障碍物', ...
    '\fontname{宋体}障碍物膨胀边缘');
h.ItemTokenSize(1) = 15;
set(h,'NumColumns',8,'location','southoutside','Box','off');
h.Position = [0.2, 0.92, 0.6, 0.1];
set(RRTC.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', ...
    1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')
set(ax1, 'Position', [0.05, 0.18, 0.45, 0.65]); % 第一个子图
set(ax2, 'Position', [0.50, 0.18, 0.45, 0.65]); % 第二个子图
fig = gcf;
fig.Units = 'centimeters';
fig.Position = [5 10 17.3 7.4]; 
% fimename = 'D:\Flycontrol\SCUT_thesis\Fig\chapter5\仿真实验地图_V2.pdf';
% PlotToFileColorPDF(RRTC,fimename,17.3,7.4);
% end