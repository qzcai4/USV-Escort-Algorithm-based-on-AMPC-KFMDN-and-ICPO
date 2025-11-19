function Initial_solution=init_MPC
    global T_MPC;
    global Enemy_number;
    global nu_limit;
    global SearchAgents_number; 
    
    ub = nu_limit(:,1);
    lb = nu_limit(:,2);
    
    Initial_solution_temp =  rand(3,T_MPC,Enemy_number,SearchAgents_number);
    mu = 3.9;  
    for j = 1:10
        for i = 1:20 
            Initial_solution_temp = sin(pi * Initial_solution_temp);
        end
        Initial_solution_temp = mu * Initial_solution_temp .* (1 - Initial_solution_temp);
    end
    Initial_solution = Initial_solution_temp.* (ub - lb) + lb;
    

end


