%% ��������
clc;
clear all;
close all;
import PID.*
INDI = 1;
%% �������壬���9������ģ��
%----------------------��ʼ������------------------------------------------------
global Fy Vci Ma Tz Tt
global m I k_T0  k_Th k_Ts sd k_Ns d_cs d_MS d_ds S den k_cpx l_cpz  Vw D   AOA  l_1 l_2 I_prop  G c_m
global  D_x D_y D_z    
global speed c1 c2 c3 c4 
global F_x F_y F_z Fm Fp Mcs Mds Mprop D_cs csAOA J T M_aero M_fan M_vane M_flap M_gyro qw
global path_Quasi_Uniform_BSpline path_len

%���ڼ�������T��ģ���ڣ�
k_T0 = 9.9796018325697625989171178675552e-6;%��ͣʱ������ϵ��
k_Th = 5.646e-06;%�����溭������ӭ�ǵ�˥��ϵ��
k_Ts = -2.247e-05;%�����溭������ӭ�ǵ�˥��ϵ��
D = 0.228;%����ֱ��
D_x = 0;%����������ɵ��ٶ�˥��������ϵ
D_y = 0;
D_z = 0;
D_x_n = 0;%����������ɵ��ٶ�˥����nedϵ
D_y_n = 0;
D_z_n = 0;

% ���ڼ��㺭�����ڷ��٣�������������
sd = 0.7;%������ѹ��
den = 1.225;%�����ܶ�kg/m^3
S = 0.040828138126052952;%���Ƚ������

d_cs = 0.0149564 * 0.7 * 0.7; %���ڼ�����������������أ���Ť�ࣩ

k_Ns = 0.0008593;%���������
d_MS = 1.1334291042e-7;%����Ť��ϵ��
d_ds = -1.3931461486e-5 / 2; %����̶�����Ť��
% d_ds = -1.3931461486e-5; %����̶�����Ť��
I_prop = 0.000037;%���ڼ�����������, ˳ʱ����ת
l_cpz = -0.055;%to be modified ���ڼ������������
k_cpx = -0.0012; %���ڼ������������

g = 9.788;
m = 1.85; %1.53;%to be modified
G = [0; 0; m * g];
l_1 = 0.17078793;%to be modified �ܸ˱۳���
l_2 = 0.06647954;%to be modified
I_x = 0.0149;%0.011866%to be modified ת������
I_y = 0.0149;%0.011866%to be modified
I_z = 0.005516;%0.00492%to be modified
% I_x = 0.011866;%to be modified
% I_y = 0.011866;%to be modified
% I_z = 0.00492;%to be modified
I = [I_x    0    0;
       0  I_y    0;
       0    0  I_z];
   
d2r = pi / 180;
r2d = 180 / pi;
c_m = 40 * d2r;%20*d2r ����޷�
uMin = [-c_m; -c_m; -c_m; -c_m];
uMax = [c_m; c_m; c_m; c_m];
O2MX = 0.6755; %��λ����������������� output 2 Mx (1���������Ӧ2�����1����)
O2MY = 0.6755;
O2MZ = 0.4985;
B = [ -0.5000,        0,    0.5000,        0;
           0,   -0.5000,         0,   0.5000;
      0.2500,    0.2500,    0.2500,   0.2500];
B_pseudo =  [  -1.0000,   -0.0000,    1.0000;
                     0,   -1.0000,    1.0000;
                1.0000,   -0.0000,    1.0000;
                     0,    1.0000,    1.0000];
alpha_filt_param.CNT = 1;
alpha_filt_param.CCR = 40;
%% ״̬������ʼ��

run('RRT_Star_Connect.m');
% data = data(1:1000, :);
% N = length(data(:, 1));
Bspline_path_diff = diff(path_Quasi_Uniform_BSpline);
Bspline_path_len = cumsum(sqrt(Bspline_path_diff(:,1).^2 + Bspline_path_diff(:,2).^2));
N = round(Bspline_path_len(end)) * 100;
Bspline_path_len = Bspline_path_len * N / Bspline_path_len(end) / 100;
time_list = linspace(0, N, size(Bspline_path_len, 1));
time_list_idx = 1;

% N = 1000; %����ʱ������λ10ms��N=1000��ʾ����10s
ts = 0.01; %������ִ��Ƶ��
time = zeros(N, 1);
for i = 1:1:N
    time(i) = i * ts; %10s ʱ����Ϊ0.01
end
X = zeros(N, 1); Y = zeros(N, 1); Z = zeros(N, 1);
V_bx = zeros(N, 1); V_by = zeros(N, 1); V_bz = zeros(N, 1);
A_x = zeros(N, 1); A_y = zeros(N, 1); A_z = zeros(N, 1);
Roll = zeros(N, 1); Pitch = zeros(N, 1); Yaw = zeros(N, 1);
Roll_fil = zeros(N, 1); Pitch_fil = zeros(N, 1); Yaw_fil = zeros(N, 1);
p = zeros(N, 1); q = zeros(N, 1); r = zeros(N, 1); r_real = zeros(N, 1);
rspeed = zeros(N, 1); V_n = zeros(N, 3); Acc_b = zeros(N, 3);
F_y1 = zeros(N, 1); F_z1 = zeros(N, 1);
F_m = zeros(N, 1); F_p = zeros(N, 1);
VCI = zeros(N, 1);
Rolld = zeros(N, 1); Pitchd = zeros(N, 1); Yawd = zeros(N, 1);
%------------------------��ʼ��״̬1-----------------------------------------
%------------------------��ʼ��״̬2----------------------------------------
% speed = 1225;
speed = 1347; %��ͣʱ����ת��
c1 = 5*d2r; c2 = 5*d2r; c3 = 5*d2r; c4 = 5*d2r;
c1 = 0 * d2r; c2 = 0 * d2r; c3 = 0 * d2r; c4 = 0 * d2r; %��Ƭƫת��ʼֵ

C1(1) = c1; C2(1) = c2; C3(1) = c3; C4(1) = c4;
Rn2b = Rn2bf(15 * d2r, 0 * d2r, 0 * d2r);
Rn2b = Rn2bf(0 * d2r, 0 * d2r, 0 * d2r);
Rb2nd = Rn2b';

f = 0;%����
f_a = 0; %����Ӧ����
omegaD = [0; 0; 0];  %���ٶ�����ֵ
omegaFdb = [0; 0; 0];  %���ٶȷ���ֵ
omegaFdbLast = [0; 0; 0];  %��һʱ�̽��ٶȷ���

alphaFilt = [0; 0; 0];  %�Ǽ��ٶ��˲�ֵ
if INDI == 0
    alphaFiltValue = [40; 40; 30];  %�Ǽ��ٶ��˲�����
    Filt_Output = [40; 40; 30];  %INDI����˲�����
else
    alphaFiltValue = [40; 40; 30];  %�Ǽ��ٶ��˲�����
    Filt_Output = [40; 40; 30];  %INDI����˲�����
    A_fil = [10; 10; 10];
end
output_i = [0; 0; 0];  %INDI���


output_f = [0; 0; 0];  %�������
output_0 = [0; 0; 0];  %���ٶȻ���һʱ������˲�ֵ
output = [0; 0; 0];  %�������/��Ƭ��������

pRef = [2; 2; -10];  %����λ��/�ο�λ��
vD = [0; -3.7; 0];  %�����ٶ�
vD = [0; 0; 0]; 
vRef = [0; 0; 0];  %�ο��ٶ�
vRefLast = [0; 0; 0];  %�ο��ٶ�
angRef = [0; 0; 0];  %�ο��Ƕ�
angVelRef = [0; 0; 0];  %�ο����ٶ�
Afil = [0; 0; -g];
Acc_sensor = [0; 0; -g];

%��ʼ״̬�� λ�á��ٶȡ���̬�����ٶ�
% Rb2n = Rn2b'; 
x0(1:3) = pRef; 
x0(4:6) = Rn2b * vD; 
x0(7:12) = [15*d2r; 0*d2r; 0*d2r; 0; 0; 0]; %״̬��ʼ��
x0(7:12) = [0 * d2r; 0 * d2r; 0 * d2r; 0; 0; 0];%״̬��ʼ��

X(1) = x0(1);
Y(1) = x0(2);
Z(1) = x0(3);
V_bx(1) = x0(4);
V_by(1) = x0(5);
V_bz(1) = x0(6);
Roll(1) = x0(7);
Pitch(1) = x0(8);
Yaw(1) = x0(9);
p(1) = x0(10);
q(1) = x0(11);
r(1) = x0(12);
%% ��������ʼ��
%-------------------------------��ʼ��������2------------------------------
K_a2 = 0; %��������ϵ��
pidRolRate = PID(0.3, 0, 0, 200);%Ki = 0.1%kd=0.5
pidPitRate = PID(0.3, 0, 0, 200);%Ki = 0.1%kd=0.5
pidYawRate = PID(0.18, 0, 0, 200);%ki = 1
% pidRolRate = PID(0.5, 0, 0, 200);%Ki = 0.1%kd=0.5
% pidPitRate = PID(0.5, 0, 0, 200);%Ki = 0.1%kd=0.5
% pidYawRate = PID(0.5, 0, 0, 200);%ki = 1

pidRol = PID(5.5, 0, 0, pi);
pidPit = PID(5.5, 0, 0, pi);
pidYaw = PID(5.0, 0.1, 0, pi);
% pidRol = PID(7.0, 0, 1, pi);
% pidPit = PID(7.0, 0, 1, pi);
% pidYaw = PID(5.0, 0, 0, pi);
if INDI == 1
pidXRate = PID(1.5, 0, 0, pi);
pidYRate = PID(1.5, 0, 0, pi);
pidZRate = PID(2.0, 0.05, 0, 200);
else
pidXRate = PID(1.5, 1, 0, pi);
pidYRate = PID(1.5, 1, 0, pi);
pidZRate = PID(2.0, 0.05, 0, 200);
end
% pidXRate = PID(4.0, 0, 0, pi);
% pidYRate = PID(4.0, 0, 0, pi);
% pidZRate = PID(5.0, 0.05, 0, 200);
% pidXRate = PID(0.1, 0, 0, 200);
% pidYRate = PID(0.1, 0, 0, 200);
% pidZRate = PID(2, 0, 0, 200);

pidX = PID(0.5, 0, 0, pi);
pidY = PID(0.5, 0, 0, pi);
pidZ = PID(2.0, 0, 0, pi);
%% ����ѭ������
for i = 1:1:N  

    tSpan = [0 ts]; %Runge-Kutta������䣬Ҳ�����ָ�ʽ���μ�matlab�����ĵ�https://ww2.mathworks.cn/help/matlab/ref/ode45.html?searchHighlight=ode45&s_tid=doc_srchtitle#bu3ugj4��
    %-----�������Ľ�Runge-Kutta�����ָ�ʽ��
    %=====% һ��[ʱ�̵�, ״̬]=ode45(���ض���΢�ַ���, �������, ��ʼ״̬, [], ����)
    %       ��[~, x]=ode45('plant', tSpan, x0, [], u);  %�ο�https://wenku.baidu.com/view/ca004bd226fff705cc170a20.html
    %=====% ����[ʱ�̵�, ״̬]=ode45(���ض���΢�ַ���, �������, ��ʼ״̬)
    %       ��[~, x]=ode45('plant', tSpan, x0);  %�ο�������򣬽����붨��Ϊglobal������
    sol = ode45(@ductedfanplant3, tSpan, x0); 
%     x0 =[X(i);Y(i);Z(i);V_bx(i);V_by(i);V_bz(i);Roll(i);Pitch(i);Yaw(i);p(i);q(i);r(i)]; %״̬����x(1),x(2)����
    [x, xdot] = deval(sol, tSpan); 
    len = length(x);
    x0 = x(:, 2);%����ϵ״̬
 %%   ���ض������
 %------------------------���ض������-----------------------------------
    X(i) = x(1, 2);
    Y(i) = x(2, 2);
    Z(i) = x(3, 2);
    V_bx(i) = x(4, 2);
    V_by(i) = x(5, 2);
    V_bz(i) = x(6, 2);

    Roll(i) = x(7, 2);
    Pitch(i) = x(8, 2);
    Yaw(i) = x(9, 2);
    p(i) = x(10, 2);
    q(i) = x(11, 2);
    r(i) = x(12, 2);
% 	Roll_fil(i) = (Roll_fil(i) * (Filt_Output(1) - 1) + Roll(i)) / Filt_Output(1);
%     Pitch_fil(i) = (Pitch_fil(i) * (Filt_Output(2) - 1) + Pitch(i)) / Filt_Output(2);
%     Yaw_fil(i) = (Yaw_fil(i) * (Filt_Output(3) - 1) + Yaw(i)) / Filt_Output(3);
%     Roll(i) = Roll_fil(i);
%     Pitch(i) = Pitch_fil(i);
%     Yaw(i) = Yaw_fil(i);
    omegaFdbLast = omegaFdb;
    Rn2b = Rn2bf(Roll(i), Pitch(i), Yaw(i));
    Rb2n = Rn2b';
    V_n(i, :) = (Rb2n * [V_bx(i); V_by(i); V_bz(i)])';
	V_xh =  V_n(i, 1) * cos(Yaw(i)) + V_n(i, 2) * sin(Yaw(i));
	V_yh = -V_n(i, 1) * sin(Yaw(i)) + V_n(i, 2) * cos(Yaw(i));
    V_n(i, 1) = V_xh;
    V_n(i, 2) = V_yh;
    Acc_b(i, :) = (Rn2b * [xdot(4, 2); xdot(5, 2); xdot(6, 2)])';
    pFdb = [X(i); Y(i); Z(i)];%λ��
    vFdb = [V_n(i, 1); V_n(i, 2); V_n(i, 3)];%�ٶ�

    omegaFdb = [p(i); q(i); r(i)];%���ٶ�
    
%     A_x(i) = Acc_b(i, 1) + g * sin(Pitch(i)) + 4 * sin(2 * pi * 1 * i / 100);
%     A_y(i) = Acc_b(i, 2) - g * cos(Pitch(i)) * sin(Roll(i));
%     A_z(i) = Acc_b(i, 3) - g * cos(Pitch(i)) * cos(Roll(i));

    A_x(i) = F_x / m;
    A_y(i) = F_y / m;
    A_z(i) = F_z / m;
    Acc_sensor = [A_x(i), A_y(i), A_z(i)];
    pRefLast = pRef;
    vRefLast = vRef;
    angRefLast = angRef;
    angVelRefLast = angVelRef;
    Rb2nd_last = Rb2nd;
%     Od_last = omegaD;
%% ��ӷ����Ŷ�
    % �������٣�����ƽ������Ϊ5 m/s
    mean_wind_speed = 0;
    % ��Ƶ���Ҳ���ģ������߶ȵĻ�������
    low_freq_component = 0 * sin(2 * pi * 0.05 * i / 100);
    % ��Ƶ���Ҳ���ģ��С�߶�����
    high_freq_component = 0 * sin(2 * pi * 1 * i / 100);
    % ���һЩ���������ģ������Ŷ�
    noise_component = 0 * randn(size(i / 100));
    % �ܷ���
    wind_speed = mean_wind_speed + low_freq_component + high_freq_component + noise_component;
    if i > 100 && i < 800
        D_x_n = wind_speed;
        D_y_n = 0;
        D_z_n = 0;
    else  if i >= 800 && i < 1500
        D_x_n = -wind_speed;
        D_y_n = 0;
        D_z_n = 0;
    else  if i >= 1500
        D_x_n = 0;
        D_y_n = 0;
        D_z_n = 0;
    end
    end
    end
    D_b= Rn2b * [D_x_n; D_y_n; D_z_n];
    D_x = D_b(1);
    D_y = D_b(2);
    D_z = D_b(3);
 %%   �⻷
%--------------------------λ�û�------------------------------------*------------------------------------------------
%   pRef = [3.5 * sin((i - 1) / 500 * 2 * pi - pi / 2) + 3.5; 
%           3.5 * cos((i - 1) / 500 * 2 * pi - pi / 2);
%                           -10                      ];%circle
%   pRef = [-3.5 * cos((i - 1) / N * 2 * pi) + 3.5
%            3.5 * sin((i - 1) / N * 2 * pi)
%                           -10               ];%circle
%   pRef = [3 * sin((i - 1) / N * 3 * pi - pi / 2) + 3;
%                 3 * sin((i - 1) / (N/2) * 2 * pi - pi);
%                         -10];
%     pRef = [i^2 / 100000; 0; -10];
%     pRef = [0; i / 100; -10];
%     pRef = [i^2 / 100000; i^2 / 100000; -10];
%     pRef = [i / 100; 0; -10];
%     pRef = [0; 0; -10 - 5*i/2000];
%     pRef = [0; 0; -10];
    if i >= time_list(time_list_idx)
        time_list_idx = time_list_idx + 1;
    end
    pRef = [path_Quasi_Uniform_BSpline(time_list_idx, 1); path_Quasi_Uniform_BSpline(time_list_idx, 2); -10];
    eP = pRef - pFdb;%λ��������λ��-״̬λ�ã�
%     eP = [0; 0; 0;];
    
    result1 = pidX.PID_Controller(eP(1), ts);
    result2 = pidY.PID_Controller(eP(2), ts);
    result3 = pidZ.PID_Controller(eP(3), ts);

    vRef = (pRef - pRefLast) / ts;%�ο��ٶ�
%     vRef = 0;
    vConstrain = 5; % ˮƽ�ٶ��޷�
    vOffset = [Constrain(result1.output, -vConstrain, vConstrain);
               Constrain(result2.output, -vConstrain, vConstrain);
               Constrain(result3.output, -1, 1)]; %�ٶȲ���
    vD = vRef + vOffset; %�����ٶ�
    vD_bx = vD(1) * cos(Yaw(i)) + vD(2) * sin(Yaw(i));
    vD_by = -vD(1) * sin(Yaw(i)) + vD(2) * cos(Yaw(i));
    vD(1) = vD_bx;
    vD(2) = vD_by;
    for j = 1:1:3
        vD(j) = Constrain(vD(j), -5, 5); %�����ٶ��޷�
    end

%     if i<50                  %�ٶ�ģʽ
%         vD(1) = 0;
%     else
%         vD(1) = 10;
%     end   
%--------------------------�ٶȻ�------------------------------------*------------------------------------------------
    accRef = (vRef - vRefLast) / ts; %�ο����ٶ�
    accRef_bx = accRef(1) * cos(Yaw(i)) + accRef(2) * sin(Yaw(i));
    accRef_by = -accRef(1) * sin(Yaw(i)) + accRef(2) * cos(Yaw(i));
    accRef(1) = accRef_bx;
    accRef(2) = accRef_by;
%     accRef = 0;
    eV = vD - vFdb; %�ٶ����
    result1 = pidXRate.PID_Controller(eV(1), ts);
    result2 = pidYRate.PID_Controller(eV(2), ts);
    result3 = pidZRate.PID_Controller(eV(3), ts);
    accOffset = [Constrain(result1.output, -3.5, 3.5);
                 Constrain(result2.output, -3.5, 3.5);
                 Constrain(result3.output, -g, g)];%���ٶȲ���
    accD = accRef + accOffset; %�������ٶ�
    
    for j = 1:1:3
        accD(j) = Constrain(accD(j), -5, 5);%���ٶ�����ֵ�޷�
    end

    %---------------------------�⻷����----------------------------------------------------------------------------------------------
%     P = Pitch(i);
%     Ya = Yaw(i);
%     R = Roll(i);
if INDI == 0
    accDisturb = [ cos(Pitch(i)) * Acc_sensor(1) + sin(Roll(i)) * sin(Pitch(i)) * Acc_sensor(2);
                     cos(Roll(i)) * Acc_sensor(2);
                    -sin(Pitch(i)) * Acc_sensor(1) + sin(Roll(i)) * cos(Pitch(i)) * Acc_sensor(2) + g];
    A = accD - [0;0;g];%accDisturb; 
else
    accDisturb = [ cos(Pitch(i)) * Acc_sensor(1) + sin(Roll(i)) * sin(Pitch(i)) * Acc_sensor(2) + cos(Roll(i)) * sin(Pitch(i)) * Acc_sensor(3);
                        cos(Roll(i)) * Acc_sensor(2) - sin(Roll(i)) * Acc_sensor(3);
                  -sin(Pitch(i)) * Acc_sensor(1) + sin(Roll(i)) * cos(Pitch(i)) * Acc_sensor(2) + cos(Roll(i)) * cos(Pitch(i)) * Acc_sensor(3) + g];

%     accDisturb = [ cos(P) * cos(Ya) * Acc_sensor(1) + (sin(R) * cos(Ya) * sin(P) - sin(Ya) * cos(R)) * Acc_sensor(2) + (cos(Ya) * cos(R) * sin(P) + sin(Ya)*sin(R)) * Acc_sensor(3)
%                        cos(P) * sin(Ya) * Acc_sensor(1) +  (sin(R) * sin(Ya) * sin(P) + cos(R) * cos(Ya)) * Acc_sensor(2) + (cos(R) * sin(Ya) * sin(P) - sin(R) * cos(Ya)) * Acc_sensor(3)
%                   -sin(P) * Acc_sensor(1) + sin(R) * cos(P) * Acc_sensor(2) + cos(R) * cos(P) * Acc_sensor(3) + g];
    A = accD - accDisturb + Afil;
    K_a2 = 0;
    for j = 1:1:3
		Afil(j) = (Afil(j) * (A_fil(j) - 1) + A(j)) / A_fil(j);
    end
end

    A_f = norm(A);
    x_c = [1; 0; 0];
%     x_c = [cos(Yaw(i)); sin(Yaw(i)); 0];
    a_c3 = -A / A_f;
    a_c2 = cross(a_c3, x_c) / norm(cross(a_c3, x_c));
    a_c1 = cross(a_c2, a_c3);
    Rb2n_f = [a_c1, a_c2, a_c3];%��̬��ת����
    Pitchd(i) = asin(Constrain(-Rb2n_f(3, 1), -1, 1));%����Pitch
	Rolld(i) = atan2(Rb2n_f(3, 2), Rb2n_f(3, 3));%����Roll
    Rb2nd = Rb2n_f;
    f_n = m * A_f;%�����������
    f_a_D = K_a2 * (A_z(i) + f_n / m);%����Ӧ����
    f_a = f_a + f_a_D * ts;%����Ӧ����
    f = f_n + f_a * m;%������
 %%   �ڻ�
    %--------------------------��̬��------------------------------------
%     Rolld(i)=20*sin(i/200*2*pi)*d2r;
%     Rolld(i)=15*d2r;
%     Rolld(i) = 0;
    Yawd(i) = 0 * d2r;
    Yawd(i) = atan2(pRef(2) - pRefLast(2), pRef(1) - pRefLast(1));
%     Yawd(i) = atan2(pRef(2) - pFdb(2), pRef(1) - pFdb(1));
%     Pitchd(i) = 0;
%     if i < 50
%         Rolld(i) = 0 * d2r;
%     else
%         Rolld(i) = 20 * d2r;
%     end
%    Rolld(i) = 1 * sin((i-1) / 500 * 2 * pi - pi / 2); 
%           3.5 * cos((i - 1) / 500 * 2 * pi - pi / 2);
%                           -10                      ];%circle

      AngleLimit = 30 * d2r; %�Ƕ������޷�
      Rolld(i) = Constrain(Rolld(i), -AngleLimit, AngleLimit); 
      Pitchd(i) = Constrain(Pitchd(i), -AngleLimit, AngleLimit); 

%     Rb2nd = Rn2bf(Rolld(i), Pitchd(i), Yawd(i)); %��̬�������
%     Rb2nd = Rb2nd';
%     Rb2n = Rn2bf(Roll(i), Pitch(i), Yaw(i));
%     Rb2n = Rb2n';
%     eAng = -0.5*vee(Rb2nd'*Rb2n - Rb2n'*Rb2nd);
    eAng = [Rolld(i) - Roll(i); Pitchd(i) - Pitch(i); Yawd(i) - Yaw(i)]; %
    eAng(3) = modPI(eAng(3));
    Yaw(i) = modPI(Yaw(i));
    % 
    % ��̬�������
    result1 = pidRol.PID_Controller(eAng(1) + 3*sin(i / 400)*d2r, ts);
    result2 = pidPit.PID_Controller(eAng(2) - 3*sin(i / 300)*d2r, ts);
    result3 = pidYaw.PID_Controller(eAng(3) - 4*cos(i / 500)*d2r, ts);
%     result1 = pidRol.PID_Controller(eAng(1) + 5*d2r, ts);
%     result2 = pidPit.PID_Controller(eAng(2) - 3*d2r, ts);
%     result3 = pidYaw.PID_Controller(eAng(3) - 2*cos(i / 500)*d2r, ts);
    result1 = pidRol.PID_Controller(eAng(1), ts);
    result2 = pidPit.PID_Controller(eAng(2), ts);
    result3 = pidYaw.PID_Controller(eAng(3), ts);
    angVelOffset = [result1.output;
                    result2.output;
                    result3.output;];%��̬�仯�ʲ���
    angVelRef = 0;%�ο���̬�仯��
    angVelD = angVelRef + angVelOffset; %������̬�仯��
%     for j = 1:1:3
%         angVelD(j) = Constrain(angVelD(j), -5, 5);%������̬�仯���޷�
%     end
    w = [1                0                 -sin(Pitch(i));  %��̬�仯�ʵ�������ٶ�
         0     cos(Roll(i))     sin(Roll(i))*cos(Pitch(i));
         0    -sin(Roll(i))    cos(Roll(i))*cos(Pitch(i))];
    omegaD = w * angVelD; %�������ٶ�
    
    %--------------------------���ٶȻ�------------------------------------
    alphaRaw = (omegaFdb - omegaFdbLast) / ts; %��ֵõ�ԭʼ�Ǽ��ٶ�
    for j = 1:1:3
        alphaFilt(j) = (alphaFilt(j) * (alphaFiltValue(j) - 1) + alphaRaw(j)) / alphaFiltValue(j);
    end
%     [alphaFilt, alpha_filt_param] = SlideFilt(alphaFilt, alphaRaw, 3, alpha_filt_param, 1);
    k_ve =  Constrain(-A_z(i) / g, 0.5, 1.5);
    G_cv(1) = (O2MX / I_x) * k_ve; %����Ч�ʾ���
	G_cv(2) = (O2MY / I_y) * k_ve;
	G_cv(3) = (O2MZ / I_z) * k_ve;
    for j = 1:1:3
		G_cv(j) = Constrain(G_cv(j), 10, 200);%�޷�Ϊ�˷�ֹ��Ϊ0
		output_i(j) = -alphaFilt(j) / G_cv(j) + output_0(j);
		output_i(j) = Constrain(output_i(j), -c_m, c_m); %INDI���
    end
    eOmega = omegaD - omegaFdb;%���ٶ����
    result1 = pidRolRate.PID_Controller(eOmega(1), ts);
    result2 = pidPitRate.PID_Controller(eOmega(2), ts);
    result3 = pidYawRate.PID_Controller(eOmega(3), ts);
    output_f = [result1.output;%/k_ve;
                result2.output;%/k_ve;
                result3.output;];%/k_ve]; %�Ǽ��ٶȲ���
    for j = 1:1:3
        output(j) = output_i(j) + output_f(j);
%         output(j) = output_f(j);
        output(j) = Constrain(output(j), -c_m, c_m);
        output_0(j) = (output_0(j) * (Filt_Output(j) - 1) + output(j)) / Filt_Output(j);
    end
%%   ����ͽ�����
    %----------------------------------------------------------------------
%      clc;
     progress = i / N * 100 %��ʾ����
     speed = sqrt(f / k_T0); %��ת��
    Vb = Rn2b * vFdb; % TODO ���Ǻ���
    Vciv = -Vb(3) / 2 + sqrt(((Vb(3) / 2)^2) + T / (sd * den * S));
    pseudo = [ -2.9276,         0,    3.7606;
				     0,   -2.9276,    3.7606;
				2.9276,         0,    3.7606;
				     0,    2.9276,    3.7606];
     %pseudo = pinv([-l_1        0    l_1      0;                            
     %    0     -l_1      0    l_1;
     %  l_2      l_2    l_2    l_2]); 
    pD_cs = 1 / (Vciv * Vciv * d_cs) * pseudo;
%     gyroopt = pD_cs * I_prop * speed * [omegaFdb(2); -omegaFdb(1); 0];
    gyroopt = 0;

%    c = two_dir_alloc_df4(B, output_i, output_f, uMin, uMax);%PCA
     c = B_pseudo * output; %α��
     c = c - gyroopt;
     c1 = c(1); c2 = c(2); c3 = c(3); c4 = c(4);

     C1(i) = c1 * r2d;
     C2(i) = c2 * r2d;
     C3(i) = c3 * r2d;
     C4(i) = c4 * r2d;
     cc(1:4, i) = [C1(i), C2(i), C3(i), C4(i)];
     SpeedT(i) = speed;
     VCI(i) = Vci;
     pd(i) = omegaD(1);
     qd(i) = omegaD(2);
     rd(i) = omegaD(3);
     v_xd(i) = vD(1);
     v_yd(i) = vD(2);
     v_zd(i) = vD(3);
%      M_ext1(i) = Ma(1);
%      M_ext2(i) = Ma(2);
%      M_ext3(i) = Ma(3);

     T(i) = -F_z;
     T_n(i) = f_n;
     T_f(i) = f;
     Xd(i) = pRef(1);
     Yd(i) = pRef(2);
     Zd(i) = pRef(3);
     Rd11(i) = Rb2nd(1, 1); Rd12(i) = Rb2nd(1, 2); Rd13(i) = Rb2nd(1, 3);
     Rd21(i) = Rb2nd(2, 1); Rd22(i) = Rb2nd(2, 2); Rd23(i) = Rb2nd(2, 3);
     Rd31(i) = Rb2nd(3, 1); Rd32(i) = Rb2nd(3, 2); Rd33(i) = Rb2nd(3, 3);
     R11(i) = Rb2n(1, 1); R12(i) = Rb2n(1, 2); R13(i) = Rb2n(1, 3);
     R21(i) = Rb2n(2, 1); R22(i) = Rb2n(2, 2); R23(i) = Rb2n(2, 3);
     R31(i) = Rb2n(3, 1); R32(i) = Rb2n(3, 2); R33(i) = Rb2n(3, 3);
     AMaero(1:3, i) = M_aero;
     AMvane(1:3, i) = M_vane;
     AMfan(1:3, i) = M_fan;
     AMflap(1:3, i) = M_flap;
     AMgyro(1:3, i) = M_gyro;

end

%% ��ͼ��ʾ
% figure(1), %λ��
% %plot(time, data(:, 4)/100, 'b');grid on;hold on;%ode45��⣬t��time��ͬ
% plot(time, X, 'k', time, Xd, 'r');grid on;hold on;
% title('������λ��X');xlabel('ʱ�䣨s��');ylabel('m')
% figure(2), 
% %plot(time, data(:, 5)/100, 'b');grid on;hold on;
% plot(time, Y, 'k', time, Yd, 'r');grid on;hold on;
% title('������λ��Y');xlabel('ʱ�䣨s��');ylabel('m')
% figure(3), 
% %plot(time, data(:, 13)/100, 'b');grid on;hold on;
% plot(time, Z, 'k', time, Zd, 'r');grid on;hold on;
% title('������λ��Z');xlabel('ʱ�䣨s��');ylabel('m')
% figure(4), %�ٶ�
% %plot(time, data(:, 10)/100, 'b');grid on;hold on;%ode45��⣬t��time��ͬ
% plot(time, V_n(:, 1), 'k', time,  v_xd, 'r');grid on;hold on;
% title('�������ٶ�Vx');xlabel('ʱ�䣨s��');ylabel('m/s')
% figure(5), 
% %plot(time, data(:, 11)/100, 'b');grid on;hold on;
% plot(time, V_n(:, 2), 'k', time,  v_yd, 'r');grid on;hold on;
% title('�������ٶ�Vy');xlabel('ʱ�䣨s��');ylabel('m/s')
% figure(6), 
% %plot(time, data(:, 14)/100, 'b');grid on;hold on;
% plot(time, V_n(:, 3), 'k', time,  v_zd, 'r');grid on;hold on;
% title('�������ٶ�Vz');xlabel('ʱ�䣨s��');ylabel('m/s')
% figure(7), 
% plot(time, Roll*r2d, 'k', time, Rolld*r2d, 'r');grid on;hold on;
% title('ŷ����Roll');xlabel('ʱ�䣨s��');ylabel('deg')
% figure(8), 
% plot(time, Pitch*r2d, 'k', time, Pitchd*r2d, 'r');grid on;hold on;
% title('ŷ����Pitch');xlabel('ʱ�䣨s��');ylabel('deg')
% figure(9), 
% %plot(time, data(:, 27), 'b');grid on;hold on;
% plot(time, Yaw*r2d, 'k', time, Yawd*r2d, 'r');grid on;hold on;
% title('ŷ����Yaw');xlabel('ʱ�䣨s��');ylabel('deg')
% 
% figure(10), %pqr
% plot(time, p*r2d, 'k',  time,  pd*r2d, 'r');grid on;hold on;
% title('���ٶ�p');xlabel('ʱ�䣨s��');ylabel('deg/s')
% figure(11), 
% %plot(time, data(:, 16)*r2d./100, 'b');grid on;hold on;
% plot(time, q*r2d, 'k', time,  qd*r2d, 'r');grid on;hold on;
% title('���ٶ�q');xlabel('ʱ�䣨s��');ylabel('deg/s')
% figure(12), 
% %plot(time, data(:, 17)*r2d./100, 'b');grid on;hold on;
% plot(time, r*r2d, 'k', time,  rd*r2d, 'r');grid on;hold on;
% title('���ٶ�r');xlabel('ʱ�䣨s��');ylabel('deg/s')
% figure
% plot(time, M_exth1, 'k', time, M_ext1, 'r');grid on;hold on;
% title('Mext1');xlabel('ʱ�䣨s��');ylabel('N.m')
% figure
% plot(time, M_exth3, 'k', time, M_ext3, 'r');grid on;hold on;
% title('Mext3');xlabel('ʱ�䣨s��');ylabel('N.m')
% figure
% plot(time, T, 'k', time, T_n, 'r', time, T_f, 'b');grid on; hold on;
% title('Thrust');xlabel('ʱ�䣨s��');ylabel('N.m')

% load('X1.mat');load('Y1.mat')
% load('oldData.mat');
%λ�ù켣

fig1 = figure
plot(Xd, Yd, 'k', 'linewidth', 1);grid on;hold on;
plot(X, Y, '--', 'color', [0, 0.45, 0.74], 'linewidth', 1);hold on;
% plot(X1, Y1, '-.', 'color', [0.85, 0.33, 0.1], 'LineWidth', 1);hold on;%'���ο���'
% axis([-1 7 -4 5]);%������������ʾ��Χ��X��-1��7��Y��-4��5��
xlabel('Pos-X(m)');
ylabel('Pos-Y(m)');
h = legend('Ref', 'Real');%legend('boxoff');
set(h, 'NumColumns', 1, 'location', 'northeast');%northwest
set(fig1.CurrentAxes,  'FontSize',  10, 'FontName', 'Times New Roman', 'LabelFontSizeMultiplier',  1, 'TitleFontSizeMultiplier', 1, 'LineWidth', 0.5, 'Xcolor', 'black', 'Ycolor', 'black', 'Zcolor', 'black')
sgtitle('PosX-PosY');

%λ��
fig1 = figure
subplot(3, 1, 1)
plot(time, Xd, 'k', 'LineWidth', 1); grid on; hold on;
plot(time, X, '--', 'color', [0, 0.45, 0.74], 'LineWidth', 1);
% axis([0 10 -1 7]);
ylabel('x(m)');
h = legend('Ref.', 'Real');
set(fig1.CurrentAxes, 'FontSize', 10, 'FontName', 'Times New Roman', 'LabelFontSizeMultiplier', 1, 'TitleFontSizeMultiplier', 1, 'LineWidth', 0.5, 'Xcolor', 'black', 'Ycolor', 'black', 'Zcolor', 'black')
subplot(3, 1, 2)
plot(time, Yd, 'k', 'LineWidth', 1);grid on;hold on;
plot(time, Y, '--', 'color', [0, 0.45, 0.74], 'LineWidth', 1);
% axis([0 10 -4 4]);
ylabel('y(m)');
h = legend('Ref.', 'Real');
set(fig1.CurrentAxes, 'FontSize', 10, 'FontName', 'Times New Roman', 'LabelFontSizeMultiplier',  1, 'TitleFontSizeMultiplier', 1, 'LineWidth', 0.5, 'Xcolor', 'black', 'Ycolor', 'black', 'Zcolor', 'black')
subplot(3, 1, 3)
plot(time, Zd, 'k', 'LineWidth', 1);grid on;hold on;
plot(time, Z, '--', 'color', [0, 0.45, 0.74], 'LineWidth', 1);
% axis([0 10 -10.2 -9.9]);
xlabel('t(s)');
ylabel('z(m)');
h = legend('Ref.', 'Real');%legend('boxoff');
set(h, 'NumColumns', 2, 'location', 'northwest');%northwest
set(fig1.CurrentAxes, 'FontSize', 10, 'FontName', 'Times New Roman', 'LabelFontSizeMultiplier',  1, 'TitleFontSizeMultiplier', 1, 'LineWidth', 0.5, 'Xcolor', 'black', 'Ycolor', 'black', 'Zcolor', 'black')
sgtitle('Position-Times');


% %�ٶ�
fig1 = figure
subplot(3, 1, 1)
plot(time, v_xd, 'k', 'LineWidth', 1);grid on;hold on;
plot(time, V_n(:, 1), '--', 'color', [0, 0.45, 0.74], 'LineWidth', 1);
% axis([0 10 -3 3]);
ylabel('v_x(m/s)');
set(fig1.CurrentAxes, 'FontSize', 10,'FontName', 'Times New Roman', 'LabelFontSizeMultiplier',  1, 'TitleFontSizeMultiplier', 1, 'LineWidth', 0.5, 'Xcolor', 'black', 'Ycolor', 'black', 'Zcolor', 'black')
subplot(3, 1, 2)
plot(time, v_yd, 'k', 'LineWidth', 1);grid on;hold on;
plot(time, V_n(:, 2), '--', 'color', [0, 0.45, 0.74], 'LineWidth', 1);
% axis([0 10 -5 5]);
ylabel('v_y(m/s)');
set(fig1.CurrentAxes, 'FontSize', 10, 'FontName', 'Times New Roman', 'LabelFontSizeMultiplier',  1, 'TitleFontSizeMultiplier', 1, 'LineWidth', 0.5, 'Xcolor', 'black', 'Ycolor', 'black', 'Zcolor', 'black')
subplot(3, 1, 3)
plot(time, v_zd, 'k', 'LineWidth', 1);grid on;hold on;
plot(time, V_n(:, 3), '--', 'color', [0, 0.45, 0.74], 'LineWidth', 1);
% axis([0 10 -0.1 0.1]);
xlabel('t(s)');
ylabel('v_z(m/s)');
h = legend('Ref.', 'Real');%legend('boxoff');
set(h, 'NumColumns', 1, 'location', 'northwest');%northwest
set(fig1.CurrentAxes, 'FontSize', 10, 'FontName', 'Times New Roman', 'LabelFontSizeMultiplier',  1, 'TitleFontSizeMultiplier', 1, 'LineWidth', 0.5, 'Xcolor', 'black', 'Ycolor', 'black', 'Zcolor', 'black')
sgtitle('Velocity-Times');

%�Ƕ�
fig1 = figure
subplot(3, 1, 1)
plot(time, Rolld*r2d, 'k', 'LineWidth', 1);grid on;hold on;
plot(time, Roll*r2d, '--', 'color', [0, 0.45, 0.74], 'LineWidth', 1);
% axis([0 5 -15 15]);
ylabel('Roll(deg)');
set(fig1.CurrentAxes, 'FontSize', 10, 'FontName', 'Times New Roman', 'LabelFontSizeMultiplier',  1, 'TitleFontSizeMultiplier', 1, 'LineWidth', 0.5, 'Xcolor', 'black', 'Ycolor', 'black', 'Zcolor', 'black')
subplot(3, 1, 2)
plot(time, Pitchd*r2d, 'k', 'LineWidth', 1);grid on;hold on;
plot(time, Pitch*r2d, '--', 'color', [0, 0.45, 0.74], 'LineWidth', 1);
% axis([0 5 -15 15]);
ylabel('Pitch(deg)');
set(fig1.CurrentAxes, 'FontSize', 10, 'FontName', 'Times New Roman', 'LabelFontSizeMultiplier',  1, 'TitleFontSizeMultiplier', 1, 'LineWidth', 0.5, 'Xcolor', 'black', 'Ycolor', 'black', 'Zcolor', 'black')
subplot(3, 1, 3)
plot(time, Yawd*r2d, 'k', 'LineWidth', 1);grid on;hold on;
plot(time, Yaw*r2d, '--', 'color', [0, 0.45, 0.74], 'LineWidth', 1);
% axis([0 5 -15 15]);
xlabel('t(s)');
ylabel('Yaw(deg)');
h = legend('Ref.', 'Real');%legend('boxoff');
set(h, 'NumColumns', 1, 'location', 'northwest');%northwest
set(fig1.CurrentAxes, 'FontSize', 10, 'FontName', 'Times New Roman', 'LabelFontSizeMultiplier',  1, 'TitleFontSizeMultiplier', 1, 'LineWidth', 0.5, 'Xcolor', 'black', 'Ycolor', 'black', 'Zcolor', 'black')
sgtitle('Attitude');


% %��ת����
% fig1 = figure
% subplot(3, 3, 1)
% plot(time,Rd11,'k','LineWidth',1);grid on;hold on;
% plot(time,R11,'--','color',[0,0.45,0.74],'LineWidth',1);
% axis([0 10 -1 1]);
% ylabel('R11');
% set(fig1.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')
% subplot(3,3,2)
% plot(time,Rd12,'k','LineWidth',1);grid on;hold on;
% plot(time,R12,'--','color',[0,0.45,0.74],'LineWidth',1);
% axis([0 10 -1 1]);
% ylabel('R12');
% set(fig1.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')
% subplot(3,3,3)
% plot(time,Rd13,'k','LineWidth',1);grid on;hold on;
% plot(time,R13,'--','color',[0,0.45,0.74],'LineWidth',1);
% axis([0 10 -1 1]);
% ylabel('R13');
% set(fig1.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')
% subplot(3,3,4)
% plot(time,Rd21,'k','LineWidth',1);grid on;hold on;
% plot(time,R21,'--','color',[0,0.45,0.74],'LineWidth',1);
% axis([0 10 -1 1]);
% ylabel('R21');
% set(fig1.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')
% subplot(3,3,5)
% plot(time,Rd22,'k','LineWidth',1);grid on;hold on;
% plot(time,R22,'--','color',[0,0.45,0.74],'LineWidth',1);
% axis([0 10 -1 1]);
% ylabel('R22');
% set(fig1.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')
% subplot(3,3,6)
% plot(time,Rd23,'k','LineWidth',1);grid on;hold on;
% plot(time,R23,'--','color',[0,0.45,0.74],'LineWidth',1);
% axis([0 10 -1 1]);
% ylabel('R23');
% set(fig1.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')
% subplot(3,3,7)
% plot(time,Rd31,'k','LineWidth',1);grid on;hold on;
% plot(time,R31,'--','color',[0,0.45,0.74],'LineWidth',1);
% axis([0 10 -1 1]);
% ylabel('R31');
% set(fig1.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')
% subplot(3,3,8)
% plot(time,Rd32,'k','LineWidth',1);grid on;hold on;
% plot(time,R32,'--','color',[0,0.45,0.74],'LineWidth',1);
% axis([0 10 -1 1]);
% xlabel('t(s)');
% ylabel('R32');
% set(fig1.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')
% subplot(3,3,9)
% plot(time,Rd33,'k','LineWidth',1);grid on;hold on;
% plot(time,R33,'--','color',[0,0.45,0.74],'LineWidth',1);
% axis([0 10 -1 1]);
% ylabel('R33');
% h = legend('Ref.','Real');%legend('boxoff');
% set(h,'NumColumns',1,'location','northwest');%northwest
% set(fig1.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')
% sgtitle('Rotation Matrix-Times');

% ���ٶ�
% fig1 = figure
% subplot(3,1,1)
% plot(time,pd.*r2d,'k','LineWidth',1);grid on;hold on;
% plot(time,p.*r2d,'--','color',[0,0.45,0.74],'LineWidth',1);
% % axis([0 10 -50 50]);
% ylabel('p(deg/s)');
% set(fig1.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')
% subplot(3,1,2)
% plot(time,qd.*r2d,'k','LineWidth',1);grid on;hold on;
% plot(time,q.*r2d,'--','color',[0,0.45,0.74],'LineWidth',1);
% % axis([0 10 -50 50]);
% ylabel('q(deg/s)');
% set(fig1.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')
% subplot(3,1,3)
% plot(time,rd.*r2d,'k','LineWidth',1);grid on;hold on;
% plot(time,r.*r2d,'--','color',[0,0.45,0.74],'LineWidth',1);
% % axis([0 10 -50 50]);
% xlabel('t(s)');
% ylabel('r(deg/s)');
% h = legend('Ref.','Real');%legend('boxoff');
% set(h,'NumColumns',1,'location','northwest');%northwest
% set(fig1.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')
% sgtitle('Angular velocity-Times');

% figure11 = figure    %ƫ��
% subplot(4,1,1);
% plot(time, C1, 'b', 'LineWidth', 1);grid on;hold on;
% ylabel('Vane1(deg)');axis([0 5 -40 40]);
% set(figure11.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')
% subplot(4,1,2);
% plot(time,C2, 'b', 'LineWidth', 1);grid on;hold on;
% ylabel('Vane2(deg)');axis([0 5 -40 40]);
% set(figure11.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')
% subplot(4,1,3);
% plot(time,C3, 'b', 'LineWidth', 1);grid on;hold on;
% ylabel('Vane3(deg)');axis([0 5 -40 40]);
% set(figure11.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')
% subplot(4,1,4);
% plot(time,C4, 'b', 'LineWidth', 1);grid on;hold on;
% ylabel('Vane4(deg)');axis([0 5 -40 40]);
% set(figure11.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')
% xlabel('Time(s)');
% legend( 'Real', 'Virtual');
% sgtitle('��Ƭƫת��');

% fig1 = figure
% subplot(4,1,1)
% plot(time,M_ext1,'k','LineWidth',1);grid on;hold on;
% plot(time,M_exth1,'--','color',[0,0.45,0.74],'LineWidth',1);
% %axis([0 10 -200 200]);
% ylabel('M_e_x_t_1(N.m)');
% set(fig1.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')
% subplot(4,1,2)
% plot(time,M_ext2,'k','LineWidth',1);grid on;hold on;
% plot(time,M_exth2,'--','color',[0,0.45,0.74],'LineWidth',1);
% %axis([0 10 -100 50]);
% ylabel('M_e_x_t_2(N.m)');
% set(fig1.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')
% subplot(4,1,3)
% plot(time,M_ext3,'k','LineWidth',1);grid on;hold on;
% plot(time,M_exth3,'--','color',[0,0.45,0.74],'LineWidth',1);
% %axis([0 10 -50 50]);
% ylabel('M_e_x_t_3(N.m)');
% h = legend('Real','Est.');%legend('boxoff');
% set(h,'NumColumns',1,'location','northwest');%northwest
% set(fig1.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')
% subplot(4,1,4)
% plot(time,T,'k','LineWidth',1);grid on;hold on;
% plot(time,T_n,'--','color',[0,0.45,0.74],'LineWidth',1);grid on;hold on;
% plot(time,T_f,'-.','color',[0,0.5,0],'LineWidth',1);
% axis([0 10 13 20]);
% xlabel('t(s)');
% ylabel('Force(N)');
% h = legend('-mA_z','f_n','f');%legend('boxoff');
% set(h,'NumColumns',1,'location','northwest');%northwest
% set(fig1.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')

function vFdb = vee(M)
    % �����Գƾ���ӳ��Ϊ����
    vFdb = [M(3, 2); M(1, 3); M(2, 1)];
end
function M_hat = hat(omegaFdb)
    % ���룺��ά���� omegaFdb = [O1, O2, O3]
    % ��������Գƾ��� M_hat

    % ȷ����������ά����
    assert(length(omegaFdb) == 3, '�����������ά����');

    % ���ɷ��Գƾ���
    M_hat = [0, -omegaFdb(3), omegaFdb(2);
             omegaFdb(3), 0, -omegaFdb(1);
             -omegaFdb(2), omegaFdb(1), 0];
end
function theta = modPI(theta)
    if theta < -pi
        while theta < -pi
            theta = theta + 2 * pi;
        end
    elseif theta > pi
        while theta > pi
            theta = theta - 2 * pi;
        end
    end
end
