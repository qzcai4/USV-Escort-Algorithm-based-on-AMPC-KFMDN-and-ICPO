function [q_new] = q_iter_innew(q,nu)
% q_iter函数：用于更新无人艇状态
% 输入量：q:当前时刻的状态量，nu：当前时刻给定的驱动量，delta_t:每次更新驱动量的时间间隔/单次系统迭代的时间，h：每次状态更新的步长，delta_t应是h的整数倍，即：delta_t=k*h。
% 过程量：q_temp:当前时刻到下一时刻的迭代过程中的状态量，随迭代次数变动并趋近于最终量。
% 输出量：q_new:下一时刻的状态量
% 检查delta_t是否为h的整数倍
global delta_t;
global h;
global Enemy_number;
% 检查delta_t是否为h的整数倍
    if mod(delta_t, h) ~= 0
        error('delta_t must be an integer multiple of h');
    end
    
    % 预分配q_new的内存
    q_new = zeros(size(q));
    
    % 迭代更新状态
    num_steps = delta_t / h;
    q_temp = q; % 初始化临时状态为当前状态
    for k = 1:num_steps
        % 向量化计算状态更新
        J = zeros(3, 3, Enemy_number);
        for i = 1:Enemy_number
            J(:,:,i) = JETA(q_temp(3,i));
        end
        q_temp = q_temp + h * reshape(sum(J .* nu(:,:,ones(1,Enemy_number)), 1), size(q_temp));
    end
    
    % 更新最终状态
    q_new = q_temp;
end

