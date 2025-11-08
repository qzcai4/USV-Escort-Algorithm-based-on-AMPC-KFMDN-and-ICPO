function [q_new] = q_iter(q,nu)

global delta_t_es;
global h;
delta_t = delta_t_es;
if mod(delta_t,h)~=0
    disp('error:delta_t/h~=0');
    return
end

    q_temp=zeros(size(q));
    for k=1:(delta_t/h)
        if (k==1)&&((delta_t/h)~=1)
            for i=1:size(nu,2) 
                q_temp(:,i)=q(:,i)+h*JETA(q(3,i))*nu(:,i) ;       
            end
        elseif k==(delta_t/h)
            for i=1:size(nu,2) 
                q_new(:,i)=q_temp(:,i)+h*JETA(q_temp(3,i))*nu(:,i) ;       
            end
        elseif (k==1)&&(k==(delta_t/h))
            for i=1:size(nu,2) 
                q_new(:,i)=q(:,i)+h*JETA(q(3,i))*nu(:,i) ;      
            end    
        else
            for i=1:size(nu,2) 
                q_temp(:,i)=q_temp(:,i)+h*JETA(q_temp(3,i))*nu(:,i) ;       
            end
        end
    end
end

