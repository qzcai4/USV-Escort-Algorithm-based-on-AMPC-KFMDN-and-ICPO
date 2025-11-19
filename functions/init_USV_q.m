function [q_i] = init_USV_q(q0,USV_number,r_safe,r_def)
q_i = zeros(3,USV_number);

if r_def==0 
    for i=1:USV_number
        r = r_safe*rand(1)+r_safe;
        theta = 2*pi*rand(1);
        q_i(1,i) = q0(1)+r*cos(theta);
        q_i(2,i) = q0(2)+r*sin(theta); 
    end
else
    theta = 2*pi/USV_number;
    r = r_def/(sin(theta/2));
    if r < r_safe
        r = r_safe;
    end
    for i = 1:USV_number
    q_i(1,i) = q0(1)+r*cos(i*theta);
    q_i(2,i) = q0(2)+r*sin(i*theta);    
    end
end

end

