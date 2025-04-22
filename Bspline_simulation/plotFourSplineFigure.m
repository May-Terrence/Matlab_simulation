% function plotFigure(map_size,startPos, goalPos, map, treeNodes,path_opt)
close all
FourSpline = figure;




hold on
grid on
% 主体图形绘制
plot(P(1,:), P(2,:),'LineWidth', 1.2,  'Color', [.3, .6, .9],'HandleVisibility', 'off');
scatter(P(1,:), P(2,:), 20,'MarkerEdgeColor','g',...
    'MarkerFaceColor','g');
plot(path_Nonuniform_BSpline(:,1),path_Nonuniform_BSpline(:,2),'LineWidth', 1.5,  'Color', 'r')
text(0.2, 0.08, '\fontname{Times New Roman}P_0', 'FontSize', 10, 'Color', 'k');
text(1.1, 1.05, '\fontname{Times New Roman}P_1', 'FontSize', 10, 'Color', 'k');
text(2.2, 0.08, '\fontname{Times New Roman}P_2', 'FontSize', 10, 'Color', 'k');
text(3.1, 1.05, '\fontname{Times New Roman}P_3', 'FontSize', 10, 'Color', 'k');
text(4.1, 0.08, '\fontname{Times New Roman}P_4', 'FontSize', 10, 'Color', 'k');

axis([0 4.5 0 1.1]);
ylabel('\fontname{宋体}距离\fontname{Times New Roman}(m)');
xlabel('\fontname{宋体}距离\fontname{Times New Roman}(m)');
h = legend('\fontname{宋体}控制点', '\fontname{Times New Roman}B\fontname{宋体}样条曲线');
h.ItemTokenSize(1) = 15;
set(h,'NumColumns',3,'location','northoutside','Box','off');
set(FourSpline.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')

% ax1 = subplot(2, 2, 1);
% ax2 = subplot(2, 2, 2);
% ax3 = subplot(2, 2, 3);
% ax4 = subplot(2, 2, 4);


% set(ax1, 'Position', [0.1, 0.6, 0.35, 0.35]); % 第一个子图
% set(ax2, 'Position', [0.6, 0.6, 0.35, 0.35]); % 第二个子图
% set(ax3, 'Position', [0.1, 0.1, 0.4, 0.35]); % 第三个子图
% set(ax4, 'Position', [0.6, 0.1, 0.4, 0.35]); % 第三个子图


% 创建一个额外的坐标轴用于放置图例
% ax = axes('Position', [0.1 0.9 0.8 0.1], 'Visible', 'off');
% 
% % 获取所有子图的图例标签
% h = findall(gcf, 'Type', 'line');
% legend(ax, h, get(h, 'DisplayName'), 'Location', 'north', 'Orientation', 'horizontal');

fig = gcf;
fig.Units = 'centimeters';
fig.Position = [5 10 8 6]; 
% fimename = 'D:\Flycontrol\SCUT_thesis\Fig\非均匀B样条曲线示意图.pdf';
% PlotToFileColorPDF(FourSpline,fimename,8,6);
% end