function [q_new] = q_iter_innew(q,nu)

global delta_t;
global h;
global Enemy_number;
    if mod(delta_t, h) ~= 0
        error('delta_t must be an integer multiple of h');
    end
    
    q_new = zeros(size(q));
    
    num_steps = delta_t / h;
    q_temp = q; 
    for k = 1:num_steps
        J = zeros(3, 3, Enemy_number);
        for i = 1:Enemy_number
            J(:,:,i) = JETA(q_temp(3,i));
        end
        q_temp = q_temp + h * reshape(sum(J .* nu(:,:,ones(1,Enemy_number)), 1), size(q_temp));
    end
    
    q_new = q_temp;
end

