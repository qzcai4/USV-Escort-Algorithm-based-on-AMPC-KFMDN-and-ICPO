function [q_new] = q_iter_in(q,nu)

global delta_t;
global h;
global Enemy_number;
if mod(delta_t,h)~=0
    disp('error:delta_t/h~=0');
    return
end
q = squeeze(q);
nu = squeeze(nu);
    q_temp=zeros(size(q));
    for k=1:(delta_t/h)
        if (k==1)&&((delta_t/h)~=1)
            for i=1:Enemy_number 
                q_temp(:,i)=q(:,i)+h*JETA(q(3,i))*nu(:,i) ;       
            end
        elseif (k~=1)&&k==(delta_t/h)
            for i=1:Enemy_number 
                q_new(:,i)=q_temp(:,i)+h*JETA(q_temp(3,i))*nu(:,i) ;       
            end
        elseif (k==1)&&(k==(delta_t/h))
            for i=1:Enemy_number 
                q_new(:,i)=q(:,i)+h*JETA(q(3,i))*nu(:,i) ;      
            end    
        else
            for i=1:Enemy_number 
                q_temp(:,i)=q_temp(:,i)+h*JETA(q_temp(3,i))*nu(:,i) ;       
            end
        end
    end
end

