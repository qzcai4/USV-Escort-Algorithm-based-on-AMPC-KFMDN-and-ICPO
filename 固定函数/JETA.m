function J_eta = JETA(psi)
% 求解USV运动学模型由驱动向量到状态向量关于时间一阶导数的映射J(eta)
    J_eta = [cos(psi) -sin(psi) 0;
             sin(psi) cos(psi)  0;
               0         0      1];
end

