close all
clear
load('D:\Flycontrol\Matlab_simulation\第五章实验\low_density_simple_data.mat')
% load('D:\Flycontrol\Matlab_simulation\第五章实验\high_density_simple_data_V2.mat')
% RRT_color = hex2rgb('#FCBB44');
% RRT_Connect_color = hex2rgb('#F1766D');
% RRT_Star_color = hex2rgb('#7A70B5');
% RRT_Star_Connect_color = hex2rgb('#839DD1');
RRT_color = 'r';
RRT_Connect_color = 'g';
RRT_Star_color = 'b';
RRT_Star_Connect_color = 'm';
x = 1:1:100;
%%
Rol = figure;   %角度
ax1 = subplot(4, 1, 1);
semilogy(x, rrt_total_time, 'Color', RRT_color, 'LineWidth', 0.75);grid on;hold on;
semilogy(x, rrt_Connect_total_time, 'Color', RRT_Connect_color, 'LineWidth', 0.75);grid on;hold on;
semilogy(x, rrt_star_total_time, 'Color', RRT_Star_color, 'LineWidth', 0.75);grid on;hold on;
semilogy(x, rrt_star_connecttotal_time, 'Color', RRT_Star_Connect_color, 'LineWidth', 0.75);grid on;hold on;
ylabel('\fontname{宋体}搜索耗时\fontname{Times New Roman}(s)');
xlabel('\fontname{宋体}实验\fontname{Times New Roman}(\fontname{宋体}次\fontname{Times New Roman})');
yticks([0.01, 0.1, 1, 10, 100]);
set(gca, 'YTick', yticks);
h = legend('\fontname{Times New Roman}RRT', '\fontname{Times New Roman}RRT\_Connect',...
    '\fontname{Times New Roman}RRT*', '\fontname{Times New Roman}C-RRT*');
h.ItemTokenSize(1) = 15;
set(h,'NumColumns',4,'location','northoutside','Box','off');
set(Rol.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')


ax2 = subplot(4, 1, 2);
semilogy(x, rrt_total_sampleNode, 'Color', RRT_color, 'LineWidth', 0.75);grid on;hold on;
semilogy(x, rrt_Connect_total_sampleNode, 'Color', RRT_Connect_color, 'LineWidth', 0.75);grid on;hold on;
semilogy(x, rrt_star_total_sampleNode, 'Color', RRT_Star_color, 'LineWidth', 0.75);grid on;hold on;
semilogy(x, rrt_star_connecttotal_sampleNode, 'Color', RRT_Star_Connect_color, 'LineWidth', 0.75);grid on;hold on;
axis([0 100 0 1450]);
ylabel('\fontname{宋体}采样点数\fontname{Times New Roman}(\fontname{宋体}个\fontname{Times New Roman})');
xlabel('\fontname{宋体}实验\fontname{Times New Roman}(\fontname{宋体}次\fontname{Times New Roman})');
yticks([10, 100, 1000, 10000]);
set(gca, 'YTick', yticks);
set(Rol.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')


ax3 = subplot(4, 1, 3);
plot(x, rrt_total_len, 'Color', RRT_color, 'LineWidth', 0.75);grid on;hold on;
plot(x, rrt_Connect_total_len, 'Color', RRT_Connect_color, 'LineWidth', 0.75);grid on;hold on;
plot(x, rrt_star_total_len, 'Color', RRT_Star_color, 'LineWidth', 0.75);grid on;hold on;
plot(x, rrt_star_connecttotal_len, 'Color', RRT_Star_Connect_color, 'LineWidth', 0.75);grid on;hold on;
axis([0 100 79 135]);
ylabel('\fontname{宋体}路径长度\fontname{Times New Roman}(m)');
xlabel('\fontname{宋体}实验\fontname{Times New Roman}(\fontname{宋体}次\fontname{Times New Roman})');
set(Rol.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')


ax4 = subplot(4, 1, 4);
plot(x, rrt_total_ang, 'Color', RRT_color, 'LineWidth', 0.75);grid on;hold on;
plot(x, rrt_Connect_total_ang, 'Color', RRT_Connect_color, 'LineWidth', 0.75);grid on;hold on;
plot(x, rrt_star_total_ang, 'Color', RRT_Star_color, 'LineWidth', 0.75);grid on;hold on;
plot(x, rrt_star_connecttotal_ang, 'Color', RRT_Star_Connect_color, 'LineWidth', 0.75);grid on;hold on;
ylabel('\fontname{宋体}路径弯曲度\fontname{Times New Roman}(rad)');
xlabel('\fontname{宋体}实验\fontname{Times New Roman}(\fontname{宋体}次\fontname{Times New Roman})');
set(Rol.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')


set(ax1, 'Position', [0.1, 0.75, 0.8, 0.17]); % 第一个子图
set(ax2, 'Position', [0.1, 0.52, 0.8, 0.17]); % 第二个子图
set(ax3, 'Position', [0.1, 0.29, 0.8, 0.17]); % 第三个子图
set(ax4, 'Position', [0.1, 0.06, 0.8, 0.17]); % 第三个子图

fig = gcf;
fig.Units = 'centimeters';
fig.Position = [0 0 16 18]; 
% fimename = 'D:\Flycontrol\SCUT_thesis\Fig\chapter5\低密度障碍物下四种算法性能对比.pdf';
% PlotToFileColorPDF(Rol,fimename,16,18);

% (sum(rrt_total_time) - sum(rrt_star_connecttotal_time))/sum(rrt_total_time)
% (sum(rrt_total_sampleNode) - sum(rrt_star_connecttotal_sampleNode))/sum(rrt_total_sampleNode)
% (sum(rrt_total_len) - sum(rrt_star_connecttotal_len))/sum(rrt_total_len)
% (sum(rrt_total_ang) - sum(rrt_star_connecttotal_ang))/sum(rrt_total_ang)
% 
% (sum(rrt_Connect_total_time) - sum(rrt_star_connecttotal_time))/sum(rrt_Connect_total_time)
% (sum(rrt_Connect_total_sampleNode) - sum(rrt_star_connecttotal_sampleNode))/sum(rrt_Connect_total_sampleNode)
% (sum(rrt_Connect_total_len) - sum(rrt_star_connecttotal_len))/sum(rrt_Connect_total_len)
% (sum(rrt_Connect_total_ang) - sum(rrt_star_connecttotal_ang))/sum(rrt_Connect_total_ang)
% 
% (sum(rrt_star_total_time) - sum(rrt_star_connecttotal_time))/sum(rrt_star_total_time)
% (sum(rrt_star_total_sampleNode) - sum(rrt_star_connecttotal_sampleNode))/sum(rrt_star_total_sampleNode)
% (sum(rrt_star_total_len) - sum(rrt_star_connecttotal_len))/sum(rrt_star_total_len)
% (sum(rrt_star_total_ang) - sum(rrt_star_connecttotal_ang))/sum(rrt_star_total_ang)
% 
% var(rrt_total_time)
% var(rrt_Connect_total_time)
% var(rrt_star_total_time)
% var(rrt_star_connecttotal_time)
% 
% var(rrt_total_sampleNode)
% var(rrt_Connect_total_sampleNode)
% var(rrt_star_total_sampleNode)
% var(rrt_star_connecttotal_sampleNode)
% 
% var(rrt_total_len)
% var(rrt_Connect_total_len)
% var(rrt_star_total_len)
% var(rrt_star_connecttotal_len)
% 
% var(rrt_total_ang)
% var(rrt_Connect_total_ang)
% var(rrt_star_total_ang)
% var(rrt_star_connecttotal_ang)
