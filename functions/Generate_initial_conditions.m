function Generate_initial_conditions()
    global q_0 nu_0 q_i USV_number T delta_t r_safe r_def;
    
    fprintf('=== Generate Main Ship Trajectory and USV Initial States ===\n');
    
    fprintf('Enter main ship initial state:\n');
    
    x0_input = input('Enter main ship initial x coordinate: ');
    if isempty(x0_input)
        error('The variable is not entered.');
    elseif ~isnumeric(x0_input)
        error('x coordinate must be a number');
    else
        x0 = x0_input;
    end
    
    y0_input = input('Enter main ship initial y coordinate: ');
    if isempty(y0_input)
        error('The variable is not entered.');
    elseif ~isnumeric(y0_input)
        error('y coordinate must be a number');
    else
        y0 = y0_input;
    end
    
    psi0_input = input('Enter main ship initial heading angle (degrees): ');
    if isempty(psi0_input)
        error('The variable is not entered.');
    elseif ~isnumeric(psi0_input)
        error('Heading angle must be a number');
    else
        psi0_deg = psi0_input;
    end
    
    psi0 = deg2rad(psi0_deg);
    
    u0_input = input('Enter main ship speed (m/s): ');
    if isempty(u0_input)
        error('The variable is not entered.');
    elseif ~isnumeric(u0_input) || u0_input < 0
        error('Speed must be a non-negative number');
    else
        u0 = u0_input;
    end
    
    fprintf('Generating main ship constant velocity trajectory...\n');
    for t = 1:T
        q_0(1, t) = x0 + u0 * cos(psi0) * (t-1) * delta_t;
        q_0(2, t) = y0 + u0 * sin(psi0) * (t-1) * delta_t;
        q_0(3, t) = psi0;
        
        nu_0(1, t) = u0;
        nu_0(2, t) = 0;
        nu_0(3, t) = 0;
    end
    
       % 生成无人艇初始状态
   q_i(:,1,:) = init_USV_q(q_0(:,1),USV_number,r_safe,r_def);
    
    fprintf('Initial state generation completed!\n');
end