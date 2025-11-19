function [nu_e] = APF(q_e,q_0,q_i)

    q_e = squeeze(q_e);
    q_i = squeeze(q_i(1:2,:,:));
    q_0 = squeeze(q_0(1:2,:));    
    
    USV_number = size(q_i,2);
    Enemy_number = size(q_e,2);

    nu_e = zeros(3,Enemy_number);
    
    att = 0.05*ones(Enemy_number,1);
    req = -100;
    e_def = 350;
    v = 25;
    omega = 25*pi/180;

    V_att = zeros(2,Enemy_number);
    r_att = zeros(Enemy_number,1);
    P_att = zeros(2,Enemy_number);
    for i=1:Enemy_number
        V_att(:,i) = q_0 - squeeze(q_e(1:2,i));
        r_att(i) = norm(V_att(:,i));
        if r_att(i) < 100 && r_att(i)>50
            att(i) = 0.10;
        elseif r_att(i) < 50
            att(i) = 0.2;
        end
        P_att(:,i) = att(i) * V_att(:,i);
    end
    
    V_req = zeros(2,Enemy_number,USV_number);
    r_req = zeros(Enemy_number,USV_number);
    for i = 1:Enemy_number
        for j = 1:USV_number
            V_req(:,i,j) = q_i(:,j) - q_e(1:2,i);
            r_req(i,j) = norm(V_req(:,i,j));
        end 
    end
    P_req = zeros(2,Enemy_number,USV_number);
    
    for i = 1:Enemy_number
        for j = 1:USV_number
            if r_req(i,j) <= e_def
                P_req1 = req * (1 / r_req(i,j) - 1 / e_def) * r_att(i)^2 / r_req(i,j)^2;
                P_req2 = req * (1 / r_req(i,j) - 1 / e_def)^2 * r_att(i);
                P_req(:,i,j) = P_req1 / r_req(i,j) * V_req(:,i,j) + P_req2 / r_att(i) * V_att(:,i);
            end     
        end
    end
    P = P_att + squeeze(sum(P_req,3));
    theta = atan2(P(2,:),P(1,:));
    delta = q_e(3,:)-theta;
    for i = 1:Enemy_number
        if r_att(i)<v
            nu_e(1,i) = r_att(i);
        else
            nu_e(1,i) = v;
        end
            nu_e(2,i) = 0;
    end
    delta(delta>omega) = omega;
    delta(delta<-omega) = -omega;
    nu_e(3,:) = - delta;
end