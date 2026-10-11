% 将NaN替换为实际数值；需要Control System Toolbox。
m  = NaN;    % 机身和固定姿态腿部的总质量，不含车轮，kg
Mw = NaN;    % 两车轮的总质量，kg
Iw = NaN;    % 两车轮绕各自轮轴的转动惯量之和，kg*m^2
I  = NaN;    % 机身和腿部整体绕合成质心的俯仰惯量，kg*m^2
l  = NaN;    % 轮轴到上述合成质心的距离，m
r  = NaN;    % 车轮半径，m
g  = 9.81;   % 重力加速度，m/s^2

% 权重自行调整，状态顺序：[位移、速度、倾角、倾角速度]
% 单位：[m、m/s、rad、rad/s]
Q = diag([1, 1, 1, 1]);
R = 1;

% 固定腿姿态、直立附近的状态方程
a = Mw + Iw/r^2 + m;
b = m*l;
J = I + m*l^2;
D = a*J - b^2;

A = [0, 1, 0, 0;
     0, 0, -b^2*g/D, 0;
     0, 0, 0, 1;
     0, 0, a*b*g/D, 0];

B = [0;
     (J/r + b)/D;
     0;
     -(a + b/r)/D];

% 连续时间LQR；tau=-K*X，tau为两轮力矩之和，单位N*m
K = lqr(A, B, Q, R);
disp('K =');
disp(K);
