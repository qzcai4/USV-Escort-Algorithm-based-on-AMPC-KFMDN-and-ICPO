function Pridict_es(nu_0,time)
    global q_0_pre;
    global notask;
    global q_i_pre; 
    global delta_t;
    global delta_t_es;
    global T_MPC;
    global T;
    global q_i_pre_temp1;
    global nu_i_temp1;
    global nu_i_pre1;
    global USV_number;
    K = delta_t/delta_t_es;
    q_0_pre_temp = zeros(3,K);
    nu_i_temp = zeros(3,K,size(notask,2));
    q_i_pre_temp = zeros(3,K,size(notask,2));
    nu_i_pre = zeros(3,T_MPC,USV_number);
    

    if (time+T_MPC)<=T
        nu_0_pre = repmat(nu_0,[1,T_MPC]);
    elseif (time+T_MPC)>T
        nu_0_pre = zeros(3,T_MPC);
        if time<T
            nu_0_pre(:,1:(T-time)) = repmat(nu_0,[1,(T-time)]);
        end
    end


    for t = 2:T_MPC
        for k=1:K
            if k ==1
                q_0_pre_temp(:,k) = q_0_pre(:,t,time)+delta_t_es*JETA(q_0_pre(3,t,time))*nu_0_pre(:,t);
                nu_i_pre(:,t,notask) = leader_follower(q_0_pre(:,t,time),nu_0_pre(:,t),squeeze(q_i_pre(:,t,notask,time)));
                q_i_pre_temp(:,k,:) = q_iter(squeeze(q_i_pre(:,t,notask,time)),squeeze(nu_i_pre(:,t,notask)));
            elseif k==K
                nu_i_temp(:,k-1,:) = leader_follower(q_0_pre_temp(:,k-1),nu_0_pre(:,t),squeeze(q_i_pre_temp(:,k-1,:)));
                q_i_pre(:,t+1,notask,time) = q_iter(squeeze(q_i_pre_temp(:,k-1,:)),squeeze(nu_i_temp(:,k-1,:)));
                nu_i_temp1(:,:,notask,time)  = nu_i_temp;
            else
                q_0_pre_temp(:,k) = q_0_pre_temp(:,k-1)+delta_t_es*JETA(q_0_pre_temp(3,k-1))*nu_0_pre(:,t);
                nu_i_temp(:,k-1,:) = leader_follower(q_0_pre_temp(:,k-1),nu_0_pre(:,t),squeeze(q_i_pre_temp(:,k-1,:)));
                q_i_pre_temp(:,k,:) = q_iter(squeeze(q_i_pre_temp(:,k-1,:)),squeeze(nu_i_temp(:,k-1,:)));
            end
        end
    end
    q_i_pre_temp1(:,:,notask,time) = q_i_pre_temp;
    nu_i_pre1(:,:,notask,time) = nu_i_pre(:,:,notask);
end
