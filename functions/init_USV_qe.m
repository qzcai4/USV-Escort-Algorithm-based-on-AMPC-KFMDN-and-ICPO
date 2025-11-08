function [q_e] = init_USV_qe(q_0,d_def,n)

    q_e = zeros(3,n);

    rng('shuffle');

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

