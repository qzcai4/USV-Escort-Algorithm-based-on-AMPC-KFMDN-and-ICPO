function [q_i_expect,form] = leader_follower_q(q_0,nu_0,q_i)

%% 声明全局变量
h=1; 
global nu_limit; 
global r_def; 
global r_safe;
global r_form; 
global dyna; 
global Enemy_number;
global USV_number;


form_number = (USV_number-Enemy_number);
alpha = dyna(1)/dyna(2);
beta = dyna(3)/dyna(2);
nu_i_new = zeros(size(q_i));
q_i_new = zeros(size(q_i));

d_i = sqrt(sum((squeeze(q_i(1:2, 1:form_number)) - squeeze(q_0(1:2))).^2, 1));
if any(d_i>r_form)
    noform = 1:form_number;
    noform(d_i <= r_form) = [];
    form = 1:form_number;
    form(noform) = [];
else
    noform = [];
    form = 1:form_number;
end
q_0_new = q_0+h*JETA(q_0(3))*nu_0(:);
if ~isempty(noform)
    nu_i_new(1, noform) = nu_limit(1);
    nu_i_new(2, noform) = 0;
    delta_x = q_0_new(1) - q_i(1, noform);
    delta_y = q_0_new(2) - q_i(2, noform);
    theta_ref = atan2(delta_y, delta_x) + 2 * pi;
    nu_i_new(3, noform) = theta_ref - q_i(3, noform);
end
if ~isempty(form)
USV_number_form = size(form,2);

if USV_number_form>2
    theta = 2*pi/USV_number_form;
    r = r_def/(sin(theta/2));
    angles = (0:USV_number_form-1) * theta;
    q_i_new(1, form) = q_0_new(1) + r * cos(angles);
    q_i_new(2, form) = q_0_new(2) + r * sin(angles);
    q_i_new(3, form) = q_0_new(3);
elseif USV_number_form==2
    r = r_safe;
    q_i_new(:,form(1)) = q_0_new+[r 0 0]';
    q_i_new(:,form(2)) = q_0_new-[r 0 0]';    
elseif USV_number_form==1
    r = r_safe;
    q_i_new(:,form(1)) = q_0_new+[r 0 0]';
end

    q_i_expect = squeeze(q_i_new(:,form));

end
end


