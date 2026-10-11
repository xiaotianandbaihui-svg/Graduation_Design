function [K, P, poles, A, B] = wheel_leg_lqr(p, Q, R, Ts)
%WHEEL_LEG_LQR 由机械参数及Q、R计算固定腿姿态的LQR反馈矩阵。
% [K,P,poles,A,B] = wheel_leg_lqr(p,Q,R,Ts)
% p的字段及单位见run_wheel_leg_lqr.m；Ts省略或为0时求连续时间解。
% X=[s;v;theta;omega]，输入tau是两轮轮轴端力矩之和，tau=-K*X。
% A、B始终为连续时间矩阵；Ts>0时K、P、poles为离散控制结果。
% 代价为积分 integral(X'*Q*X + tau'*R*tau)dt，时域为[0,+inf)。
% Ts>0时使用lqrd精确转换该积分代价，输入在每个周期内保持常数。

if nargin < 4
    Ts = 0;
end
assert(isstruct(p) && isscalar(p), 'p必须是机械参数结构体。');
names = {'m', 'Mw', 'Iw', 'I', 'l', 'r', 'g'};
for i = 1:numel(names)
    name = names{i};
    assert(isfield(p, name), '缺少机械参数p.%s。', name);
    validateattributes(p.(name), {'numeric'}, ...
        {'real','finite','scalar','nonnegative'}, mfilename, ['p.' name]);
end
assert(p.m > 0 && p.Mw > 0 && p.l > 0 && p.r > 0 && p.g > 0, ...
    '质量、质心距离、轮半径及重力加速度必须大于0。');
validateattributes(Ts, {'numeric'}, {'real','finite','scalar','nonnegative'});
validateattributes(Q, {'numeric'}, {'real','finite','size',[4 4]});
validateattributes(R, {'numeric'}, {'real','finite','scalar','positive'});
Q = double(Q);
R = double(R);
assert(norm(Q-Q.', 'fro') <= 1e-12*max(1,norm(Q,'fro')), 'Q必须是对称矩阵。');
Q = (Q+Q.')/2;
assert(min(eig(Q)) >= 0, 'Q必须为半正定矩阵；对角权重不能小于0。');

% 直立平衡点的牛顿模型，保留电机对上部刚体的反作用力矩-tau。
a = p.Mw + p.Iw/p.r^2 + p.m;
b = p.m*p.l;
J = p.I + p.m*p.l^2;  % p.I是质心惯量，此处只加一次平行轴项
D = a*J-b^2;
assert(isfinite(D) && D > 0, '质量矩阵退化，请检查机械参数与单位。');
A = [0, 1, 0, 0;
     0, 0, -b^2*p.g/D, 0;
     0, 0, 0, 1;
     0, 0, a*b*p.g/D, 0];
B = [0; (J/p.r+b)/D; 0; -(a+b/p.r)/D];
assert(all(isfinite(A(:))) && all(isfinite(B(:))), 'A、B含无效值，请检查参数。');

if Ts == 0
    [K, P, poles] = lqr(A, B, Q, R);
    assert(all(real(poles) < 0), '连续闭环不稳定，请检查模型和Q、R。');
else
    [K, P, poles] = lqrd(A, B, Q, R, Ts);
    assert(all(abs(poles) < 1), '离散闭环不稳定，请检查模型、Q、R和周期。');
end
end
