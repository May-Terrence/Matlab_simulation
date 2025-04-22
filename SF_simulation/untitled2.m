clear
% close all
r2d = 180/pi;
% load('D:\Flycontrol\MATLAB\plot\第三章实验\simulation_INDIrvsPIDr.mat')
% load('sim_fly_PID.mat')
% load('sim_fly_PIDr.mat')
% load('sim_fly_PIDr_TEST.mat')%cha
% load('sim_fly_PIDrr_TEST.mat')%zhong
% load('sim_fly_PIDrrr_TEST.mat') %hao


% load('sim_PID_with_disturb.mat')%zhong
% load('sim_PID_with_distur.mat')%zhong
load('sim_PID_without_disturb.mat')%zhong
% load('sim_PID_without_disturbb.mat')%zhong

% load('sim_fly_PIDr_TEST_bigI.mat')%zhong

% load('sim_PID_dd.mat')%d = 3
% load('sim_PID_ddd.mat')%d = 1
% load('sim_PID_dddd.mat')%d = 0.1
% load('sim_PID_d.mat')%d = 0.5
% load('sim_PID_ddddd.mat')%d = 0.01

% load('sim_PID_cd.mat')%d = 0.01
% load('sim_PID_cddddd.mat')%d = 0.1
% load('sim_PID_cddd.mat')%d = 0.3
% load('sim_PID_cdd.mat')%d = 0.6
% load('sim_PID_cdddd.mat')%d = 1


% load('sim_PID_id.mat')%d = 0.005 i = 2
% load('sim_PID_pd.mat')%d = 1 p = 7

pidRol = Roll;
pidPit = Pitch;
pidYaw = Yaw;
% pidINDI_i = INDI_i;
% pidINDI_f = INDI_f;
% pidINDI = INDI_i + INDI_f;
% load('sim_fly.mat')
% load('sim_INDI_with_disturb.mat')
load('sim_INDI_without_disturb.mat')
% load('sim_fly_nor.mat')
% load('sim_fly_norr.mat')
% load('sim_fly_r.mat')
load('sim_fly_r_TEST.mat')
% pidINDI_i = INDI_i;
% pidINDI_f = INDI_f;
INDI = INDI_i + INDI_f;
x = linspace(0.01,5.5,550);

% save('sim_fly_r_TEST.mat','Rolld','Roll','Pitchd','Pitch','Yawd','Yaw','INDI_i','INDI_f')

%
Rol = figure   %角度

ax1 = subplot(3, 1, 1);
plot(x, (Rolld - Rolld(20))*r2d,  'k--', 'LineWidth', 0.75);grid on;hold on;
plot(x, Roll*r2d, 'Color','[0 0 0.990]', 'LineWidth', 0.75);grid on;hold on;
plot(x, pidRol*r2d, 'Color','[0.800 0 0]', 'LineWidth', 0.75);grid on;hold on;
% plot(x, pidINDI_i(1,:),'m', 'LineWidth', 0.75);
% plot(x, pidINDI_f(1,:),'g', 'LineWidth', 0.75);
% plot(x, pidINDI(1,:),'m', 'LineWidth', 0.75);
% plot(x, INDI(1,:),'g', 'LineWidth', 0.75);
% axis([0 5.5 -13 13]);
axis([0 5.5 -11 11]);
ylabel('\fontname{宋体}滚转角\fontname{Times New Roman}(°)');
xlabel('\fontname{宋体}时间\fontname{Times New Roman}(s)');
set(gca, 'XTick', [0, 1, 2, 3, 4, 5]);
set(gca, 'XTickMode', 'manual');
h = legend('\fontname{宋体}参考值', 'INDI', 'PID');
h.ItemTokenSize(1) = 20;
set(h,'NumColumns',3,'location','northoutside','Box','off');
% set(h,'NumColumns',1,'location','southwest','Box','off');
% xticks([0, 1, 2, 3, 4]);
% yticks([-1, 0, 1]);
set(Rol.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')


ax2 = subplot(3, 1, 2);
% Pit = figure   %角度
plot(x, (Pitchd - Pitchd(20))*r2d,  'k--', 'LineWidth', 0.75);grid on;hold on;
plot(x, Pitch*r2d, 'Color','[0 0 0.990]', 'LineWidth', 0.75);grid on;hold on;
plot(x, pidPit*r2d, 'Color','[0.800 0 0]', 'LineWidth', 0.75);grid on;hold on;
% plot(x, pidINDI_i(2,:),'m', 'LineWidth', 0.75);
% plot(x, pidINDI_f(2,:),'g', 'LineWidth', 0.75);
% plot(x, pidINDI(2,:),'m', 'LineWidth', 0.75);
% plot(x, INDI(2,:),'g', 'LineWidth', 0.75);
% axis([0 5.5 -8 8]);
axis([0 5.5 -1 1]);
ylabel('\fontname{宋体}俯仰角\fontname{Times New Roman}(°)');
xlabel('\fontname{宋体}时间\fontname{Times New Roman}(s)');
xticks([0, 1, 2, 3, 4, 5]);
set(Rol.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')
% h = legend('\fontname{宋体}参考值', 'INDI', 'PID');
% h.ItemTokenSize(1) = 10;
% set(h,'NumColumns',1,'location','southwest','Box','off');

% Yaw = figure   %角度
ax3 = subplot(3, 1, 3);
plot(x, (Yawd - Yawd(20))*r2d,  'k--', 'LineWidth', 0.75);grid on;hold on;
plot(x, Yaw*r2d, 'Color','[0 0 0.990]', 'LineWidth', 0.75);grid on;hold on;
plot(x, pidYaw*r2d, 'Color','[0.800 0 0]', 'LineWidth', 0.75);grid on;hold on;
% plot(x, pidINDI_i(3,:),'m', 'LineWidth', 0.75);
% plot(x, pidINDI_f(3,:),'g', 'LineWidth', 0.75);
% plot(x, pidINDI(3,:),'m', 'LineWidth', 0.75);
% plot(x, INDI(3,:),'g', 'LineWidth', 0.75);
% axis([0 5.5 -8 10]);
% axis([0 5.5 -8 15]);
% axis([0 5.5 -1 1]);
axis([0 5.5 -2 2]);
% xlabel('Time(s)');
% ylabel('Yaw(deg)');
ylabel('\fontname{宋体}偏航角\fontname{Times New Roman}(°)');
xlabel('\fontname{宋体}时间\fontname{Times New Roman}(s)');
xticks([0, 1, 2, 3, 4, 5]);

set(Rol.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')
% h = legend('\fontname{宋体}参考值', 'INDI', 'PID');
% h.ItemTokenSize(1) = 10;
% set(h,'NumColumns',1,'location','southwest','Box','off');

set(ax1, 'Position', [0.1, 0.73, 0.8, 0.20]); % 第一个子图
set(ax2, 'Position', [0.1, 0.42, 0.8, 0.20]); % 第二个子图
set(ax3, 'Position', [0.1, 0.11, 0.8, 0.20]); % 第三个子图
fig = gcf;
fig.Units = 'centimeters';
fig.Position = [0 1 14 10]; 
% fimename = 'D:\Flycontrol\SCUT_thesis\Fig\chapter3\INDI对比PID有扰动仿真实验结果_V2.pdf';
% fimename = 'D:\Flycontrol\SCUT_thesis\Fig\chapter3\INDI对比PID有扰动飞行试验结果_V2.pdf';
% PlotToFileColorPDF(Rol,fimename,14,10);
% end

% figure
% 
% plot(x,SpeedT)
% grid on;


% load('sim_duo_PID.mat')
% one = cc(1,:);
% two = cc(2,:);
% three = cc(3,:);
% four = cc(4,:);
% load('sim_duo_INDI.mat')
% x = linspace(0.01,5.5,550);
% Rol = figure   %角度
% ax1 = subplot(4, 1, 1);
% plot(x, cc(1,:), 'Color','[0 0 0.990]', 'LineWidth', 0.75);grid on;hold on;
% plot(x, one, 'Color','[0.800 0 0]', 'LineWidth', 0.75);grid on;hold on;
% % axis([0 5.5 -13 13]);
% axis([0 5.5 -20 40]);
% set(gca, 'XTick', [0, 1, 2, 3, 4, 5]);
% set(gca, 'XTickMode', 'manual');
% ylabel('\fontname{宋体}1号舵转角\fontname{Times New Roman}(°)');
% xlabel('\fontname{宋体}时间\fontname{Times New Roman}(s)');
% 
% h = legend( '\fontname{Times New Roman}INDI', '\fontname{Times New Roman}PID');
% h.ItemTokenSize(1) = 20;
% set(h,'NumColumns',2,'location','northoutside','Box','off');
% set(Rol.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')
% 
% 
% ax2 = subplot(4, 1, 2);
% plot(x, cc(2,:), 'Color','[0 0 0.990]', 'LineWidth', 0.75);grid on;hold on;
% plot(x, two, 'Color','[0.800 0 0]', 'LineWidth', 0.75);grid on;hold on;
% % axis([0 5.5 -13 13]);
% axis([0 5.5 -6 1]);
% ylabel('\fontname{宋体}2号舵转角\fontname{Times New Roman}(°)');
% xlabel('\fontname{宋体}时间\fontname{Times New Roman}(s)');
% set(gca, 'XTick', [0, 1, 2, 3, 4, 5]);
% set(gca, 'XTickMode', 'manual');
% set(Rol.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')
% 
% ax3 = subplot(4, 1, 3);
% plot(x, cc(3,:), 'Color','[0 0 0.990]', 'LineWidth', 0.75);grid on;hold on;
% plot(x, three, 'Color','[0.800 0 0]', 'LineWidth', 0.75);grid on;hold on;
% % axis([0 5.5 -13 13]);
% axis([0 5.5 -40 40]);
% ylabel('\fontname{宋体}3号舵转角\fontname{Times New Roman}(°)');
% xlabel('\fontname{宋体}时间\fontname{Times New Roman}(s)');
% set(gca, 'XTick', [0, 1, 2, 3, 4, 5]);
% set(gca, 'XTickMode', 'manual');
% set(Rol.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')
% 
% ax4 = subplot(4, 1, 4);
% plot(x, cc(4,:), 'Color','[0 0 0.990]', 'LineWidth', 0.75);grid on;hold on;
% plot(x, four, 'Color','[0.800 0 0]', 'LineWidth', 0.75);grid on;hold on;
% 
% axis([0 5.5 -5 2]);
% ylabel('\fontname{宋体}4号舵转角\fontname{Times New Roman}(°)');
% xlabel('\fontname{宋体}时间\fontname{Times New Roman}(s)');
% set(gca, 'XTick', [0, 1, 2, 3, 4, 5]);
% set(gca, 'XTickMode', 'manual');
% set(Rol.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')
% 
% set(ax1, 'Position', [0.1, 0.75, 0.8, 0.18]); % 第一个子图
% set(ax2, 'Position', [0.1, 0.52, 0.8, 0.18]); % 第二个子图
% set(ax3, 'Position', [0.1, 0.29, 0.8, 0.18]); % 第三个子图
% set(ax4, 'Position', [0.1, 0.06, 0.8, 0.18]); % 第三个子图
% 
% fig = gcf;
% fig.Units = 'centimeters';
% fig.Position = [0 0 16 20]; 
