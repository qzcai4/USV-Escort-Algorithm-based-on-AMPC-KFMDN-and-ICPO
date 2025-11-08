function Parameter_setting()
    global Enemy_number_max t_start T T_MPC delta_t delta_t_es h USV_number t_end;
    global Enemy_number Enemy_label SearchAgents_number Max_iteration;
    global nu_limit r_def r_safe d_def r_form dyna task notask;
    global q_i_gwo nu_i_gwo fitness Best_score J J_best Scenario;
    global Scenario_alpha Scenario_alpha_final t_cal Best_pos q_i_pre;
    global q_0_pre nu_i_pre1 Q R P Kal EstimateValue PredictValue;
    global KalmanValue EstimateValueAlpha PredictValueAlpha KalmanValueAlpha;
    global alphaValue HistoryPredictionValue q_e q_i nu_i USV_status;
    global task_flag D_ei D_ei1 task_check task_alltime Event_trigger;

    fprintf('=== Simulation Parameter Setting ===\n');
    
    T_input = input('Please enter the total simulation time T (unit: s): ');
    if isempty(T_input)
error("The variable is not entered.");
    elseif ~isnumeric(T_input) || T_input <= 0
        error('Total simulation time must be a positive number');
    else
        T = T_input;
    end
    
    T_MPC_input = input('Please enter the number of MPC intervals T_MPC: ');
    if isempty(T_MPC_input)
error("The variable is not entered.");
    elseif ~isnumeric(T_MPC_input) || T_MPC_input <= 0 || mod(T_MPC_input,1) ~= 0
        error('Number of MPC intervals must be a positive integer');
    else
        T_MPC = T_MPC_input;
    end
    


    
    Enemy_number_max_input = input('Please enter the maximum number of enemy ships Enemy_number_max: ');
    if isempty(Enemy_number_max_input)
error("The variable is not entered.");
    elseif ~isnumeric(Enemy_number_max_input) || Enemy_number_max_input <= 0 || mod(Enemy_number_max_input,1) ~= 0
        error('Maximum number of enemy ships must be a positive integer');
    else
        Enemy_number_max = Enemy_number_max_input;
    end
    
    SearchAgents_number_input = input('Please enter the population size of ICPO SearchAgents_number: ');
    if isempty(SearchAgents_number_input)
error("The variable is not entered.");
    elseif ~isnumeric(SearchAgents_number_input) || SearchAgents_number_input <= 0 || mod(SearchAgents_number_input,1) ~= 0
        error('Population size must be a positive integer');
    else
        SearchAgents_number = SearchAgents_number_input;
    end
    
    Max_iteration_input = input('Please enter the maximum number of iterations for ICPO Max_iteration: ');
    if isempty(Max_iteration_input)
error("The variable is not entered.");
    elseif ~isnumeric(Max_iteration_input) || Max_iteration_input <= 0 || mod(Max_iteration_input,1) ~= 0
        error('Maximum number of iterations must be a positive integer');
    else
        Max_iteration = Max_iteration_input;
    end
    
    fprintf('\n=== Velocity Vector Limits Setting ===\n');
    fprintf('Please enter the velocity vector limits nu_limit (3x2 matrix):\n');
    fprintf('Format: [surge_upper, sway_upper, yaw_rate_upper; surge_lower, sway_lower, yaw_rate_lower]''\n');
    
    nu_limit_input = input('Enter nu_limit: ');
    if isempty(nu_limit_input)
        error('The variable nu_limit is not entered.');
    elseif ~isnumeric(nu_limit_input) || ~isequal(size(nu_limit_input), [3, 2])
        error('nu_limit must be a 3x2 numeric matrix');
    else
        nu_limit = nu_limit_input;
    end

    fprintf('\n=== Enemy Ship Appearance Time Setting ===\n');
    fprintf('Choose input method for t_start:\n');
    fprintf('1. Enter a range [min, max] to generate random times\n');
    fprintf('2. Enter all appearance times directly\n');
    
    method_input = input('Enter choice (1 or 2): ');
    if isempty(method_input)
        error('The variable t_start is not entered.');
    end
    
    if method_input == 1
        range_input = input('Enter time range [min_time, max_time]): ');
        if isempty(range_input) || ~isnumeric(range_input) || numel(range_input) ~= 2 || range_input(1) >= range_input(2)
            error('Invalid time range. Please enter a valid [min, max] range');
        else
            t_start = randi([range_input(1), range_input(2)], Enemy_number_max, 1);
        end
    elseif method_input == 2
        times_input = input(['Enter all appearance times as a vector of length ', num2str(Enemy_number_max), ': ']);
        if isempty(times_input) || ~isnumeric(times_input) || numel(times_input) ~= Enemy_number_max
            error(['Please enter exactly ', num2str(Enemy_number_max), ' appearance times']);
        else
            t_start = times_input(:); 
        end
    else
        error('Invalid choice. Please enter 1 or 2');
    end
    
    fprintf('Initializing variables...\n');

    Enemy_number = 0;
    r_def = 15;
    delta_t = 1;
    h = 0.002;
    USV_number = 6;
    r_safe = 20;
    dyna = [1000,0.6,0.59];
    d_def = 1000;
    r_form = 40;
    task = zeros(Enemy_number,1);
    notask = setdiff(1:USV_number,task);
    
    q_i_gwo = zeros(3,T_MPC+1,USV_number,SearchAgents_number,T,Max_iteration);
    nu_i_gwo = zeros(3,T_MPC,USV_number,SearchAgents_number,T,Max_iteration);
    fitness = zeros(T,SearchAgents_number,Max_iteration);
    Best_score = zeros(T,Max_iteration);
    J = zeros(Enemy_number_max,T_MPC,5,T,SearchAgents_number,Max_iteration);
    J_best = zeros(Enemy_number_max,T_MPC,5,T);
    Scenario = zeros(3,T_MPC+1,Enemy_number_max,SearchAgents_number,T,Max_iteration);
    Scenario_alpha = zeros(3,T_MPC+1,Enemy_number_max,T,Max_iteration);
    Scenario_alpha_final = zeros(3,T_MPC+1,Enemy_number_max,T);
    t_cal = [];
    Best_pos = zeros(3,USV_number,T);
    q_i_pre = zeros(3,T_MPC+1,USV_number,T);
    q_0_pre = zeros(3,T_MPC+1,T);
    nu_i_pre1 = zeros(3,T_MPC,USV_number,T);
    
    Q = repmat(eye(2),[1,1,Enemy_number_max]);
    R = repmat(eye(2),[1,1,Enemy_number_max]);
    P = repmat(eye(2),[1,1,Enemy_number_max]);
    Kal = repmat(eye(2),[1,1,Enemy_number_max]);
    
    EstimateValue = zeros(2,T_MPC,Enemy_number_max,T,SearchAgents_number,Max_iteration);
    PredictValue = zeros(2,T_MPC,Enemy_number_max,T,SearchAgents_number,Max_iteration);
    KalmanValue = zeros(2,T_MPC,Enemy_number_max,T,SearchAgents_number,Max_iteration);
    EstimateValueAlpha = zeros(2,T_MPC,Enemy_number_max,T);
    PredictValueAlpha = zeros(2,T_MPC,Enemy_number_max,T);
    KalmanValueAlpha = zeros(2,T_MPC,Enemy_number_max,T);
    alphaValue = zeros(2,T);
    HistoryPredictionValue = zeros(2,2);
    
    q_e = inf(3,T,Enemy_number_max);
    q_i = zeros(3,T,USV_number);
    nu_i = zeros(3,T,USV_number);
    USV_status = zeros(USV_number,1);
    task_flag = zeros(Enemy_number_max,T);
    D_ei = zeros(USV_number,Enemy_number_max,T);
    D_ei1 = zeros(USV_number,Enemy_number_max,T);
    task_check = zeros(Enemy_number_max,2,T);
    task_alltime = zeros(USV_number,T);
    Event_trigger = zeros(T,1);
    t_end = T*ones(Enemy_number_max,1);
    delta_t_es = 0.01;
    fprintf('Parameter setting completed!\n');
end