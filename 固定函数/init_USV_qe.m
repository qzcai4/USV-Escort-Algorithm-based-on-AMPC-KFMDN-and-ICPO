function [q_e] = init_USV_qe(q_0,d_def,n)
% init_USV_q:生成所有无人艇的初始时刻状态，具备按照一定规则随机生成或以最优护卫性能固定生成两种模式
% 输入变量：q0：主舰的初始位置,USV_number：无人艇数量,r_safe：无人艇与主舰的安全距离,
%           r_def：无人艇有效拦截距离，当输入值为0时，选择随机生成满足要求的无人艇位置。
% 输出变量：q_i(3,USV_number)：所有无人艇初始时刻的状态信息

% 初始化
    q_e = zeros(3,n);

    rng('shuffle');

% 随机在距主舰感知范围极限的1-1.2倍距离范围内生成敌舰初始位置
    for i=1:n
        r1 = d_def+0.2*d_def*rand(1);
        theta1 = -5/12*pi+5/3*pi*rand(1);
        r2 = d_def+0.2*d_def*rand(1);
        theta2 = -5/12*pi+5/3*pi*rand(1);
        q_e(1,i) = q_0(1)+r1*cos(theta1);
        q_e(2,i) = q_0(2)+r2*sin(theta2);
        q_e(3,i) = atan2(q_0(2)-q_e(2,i),q_0(1)-q_e(1,i));
    end
end

