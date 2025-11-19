function [q_new] = q_iter_0(q,nu)

global delta_t;
global h;
if mod(delta_t,h)~=0
    disp('error:delta_t/h~=0');
    return
end
q = squeeze(q);
nu = squeeze(nu);

q_temp=zeros(size(q));
for k=1:(delta_t/h)
    if (k==1)&&((delta_t/h)~=1)
        q_temp(:)=q(:)+h*JETA(q(3))*nu(:) ;
    elseif (k~=1)&&k==(delta_t/h)
        q_new(:)=q_temp(:)+h*JETA(q_temp(3))*nu(:) ;
    elseif (k==1)&&(k==(delta_t/h))
        q_new(:)=q(:)+h*JETA(q(3))*nu(:) ;
    else
        q_temp(:)=q_temp(:)+h*JETA(q_temp(3))*nu(:) ;
    end
end
end

