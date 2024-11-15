    % 被控对象数学模型，两种格式：
%=====% 一：状态微分=plant(时间,状态,[],输入)
%       即dx = plant(~,x,~,u)。对应[~,x]=ode45('plant',tSpan,x0,[],u); 参考https://wenku.baidu.com/view/ca004bd226fff705cc170a20.html
%=====% 二：状态微分=ode45(时间,状态)
%       即dx = plant(~,x)。对应[~,x] = ode45('plant',tSpan,x0); 参考往届程序，将输入定义为global变量，隐含假定一个周期内u不变。
function  dx = ductedfanplant3(t, x)  
    global Fy Vci Ma Tz Tt
    global m I k_T0  k_Th k_Ts sd k_Ns d_cs  d_MS d_ds l_1 S den k_cpx l_cpz  Vw D  AOA  l_2 I_prop  G c_m
    global  D_x D_y D_z    
    global speed c1 c2 c3 c4 
    global F_x F_y F_z  Mcs Mds Mprop D_cs csAOA J T M_aero M_fan M_vane M_flap M_gyro 
    c = [c1; c2; c3; c4];%舵的偏角
    for i = 1:1:4;
        c(i) = Constrain(c(i), -c_m, c_m);%舵限幅
    end
    dx = zeros(12, 1);
    X = x(1);
    Y = x(2);
    Z = x(3);
    P = [X; Y; Z];
    u = x(4);
    v = x(5);
    w = x(6);

    V_b = [u; v; w];
    Roll = x(7);%当前姿态
    Pitch = x(8);
    Yaw = x(9);
    p = x(10);
    q = x(11);
    r = x(12);
    W = [p; q; r];
    Rn2b = Rn2bf(Roll, Pitch, Yaw);
    Rb2n = Rn2b';
    V_n = Rb2n * V_b;%NED下当前速度
    Q = [1    sin(Roll)*tan(Pitch)    cos(Roll)*tan(Pitch);  %机体角速度到姿态变化率
         0        cos(Roll)               -sin(Roll);
         0    sin(Roll)*sec(Pitch)    cos(Roll)*sec(Pitch)];
    %%  空速，迎角，前进比
    Vw = sqrt((u - D_x)^2 + (v - D_y)^2 + (w - D_z)^2); %空速：飞行器相对于气流的速度
    if (Vw == 0)
        AOA = pi / 2;%涵道本地迎角
        csAOA = 0;
    else
        AOA = acos(-(w - D_z) / Vw);
        csAOA = -(w - D_z) / Vw;
    end
    J = Vw / (speed / (2 * pi) * D); %J = pi * Va / RΩ风扇前进比                                 
    %%
    %------------------------------------计算合力F----------------------------------------------- 
    %拉力与重要变量-------------------------------------------------------------
%     T = (k_T0 + J * (k_Th + k_Ts * csAOA)) * speed^2;
    T = k_T0 * speed^2;
    Vci = -(w - D_z) / 2 + sqrt(((w - D_z) / 2)^2 + T / (sd * den * S));% 涵道出口风速

    k_cs1 = d_cs;
    k_cs2 = d_cs;
    k_cs3 = d_cs;
    k_cs4 = d_cs;
    K_cs = Vci^2 * [    0  -k_cs2       0  k_cs4;                                    %?
                    k_cs1       0  -k_cs3      0;
                        0       0       0      0];
    F_T = [0; 0; -T];%风扇拉力
    F_cs = K_cs * c;%舵面气动力
    %==================================

     F_m = [ -k_Ns * (u - D_x) * speed;
             -k_Ns * (v - D_y) * speed;
                       0           ];
    F = F_T + F_m;
    F_x = F(1);
    F_y = F(2);
    F_z = F(3);
    %----------------------------合力矩------------------------------------------
    %------------面对舵机力臂，逆时针转为正----------------------------
    M_fan = [0; 0; d_MS * speed^2];%风扇扭矩+
    %======================================================
    D_cs = Vci^2 * [-k_cs1 * l_1          0       k_cs3 * l_1         0;                            %?
                        0           -k_cs2 * l_1       0         k_cs4 * l_1;
                      k_cs1 * l_2      k_cs2 * l_2    k_cs3 * l_2    k_cs4 * l_2];           
    %===============================================
    M_flap = [0; 0; Vci * speed * d_ds];%涵道平衡反扭矩-  
    %============================================
    M_gyro = I_prop * speed * [-q; p; 0];%陀螺力矩                                       ?
    %=========================================
    M_vane = D_cs * c;%舵面力矩

    l_cpx = k_cpx * (v - D_y);
    l_cpy = k_cpx * (u - D_x);
    M_aero = [ F_m(2) * l_cpz - F_T(3) * l_cpx;                                 
              -F_m(1) * l_cpz + F_T(3) * l_cpy;
                          0                   ];
    M = M_vane + M_flap + M_fan + M_aero + M_gyro;
    % M = M_cs;
    % Mcs = M_vane(3);
    % Mds = M_flap(3);
    % Mprop = M_fan(3);
    % Ma = M - M_vane;%除舵面力矩外的其他力矩

    dx(1:3) = V_n;
    dx(4:6) = ( F + Rn2b * G )./m - cross(W, V_b);
    dx(7:9) = Q * W;
    dx(10:12) = I \ (M - cross(W,(I * W)));
end
% function  dx = plant(~,x,~,u)
% dx=zeros(6,1);
% dx(1) = x(4);
% dx(2) = x(5);
% dx(3) = x(6);
% dx(4) = u(1);
% dx(5) = u(2);
% dx(6) = u(3);
% end