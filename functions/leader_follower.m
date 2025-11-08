function [nu_i_new] = leader_follower(q_0,nu_0,q_i)
global h; 
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

    q_i_expect = q_i_new;


delta_i = zeros(3,USV_number_form);
delta_i_expect = zeros(3,USV_number_form);
ex = zeros(USV_number_form,1);
ey = zeros(USV_number_form,1);
f1 = zeros(USV_number_form,1);
f2 = zeros(USV_number_form,1);
z1 = zeros(USV_number_form,1);
z2 = zeros(USV_number_form,1);
for i = 1:USV_number_form  
    delta_i(1,i) = -(q_0_new(1)-q_i(1,form(i)))*cos(q_0_new(3))-(q_0_new(2)-q_i(2,form(i)))*sin(q_0_new(3));
    delta_i(2,i) = (q_0_new(1)-q_i(1,form(i)))*sin(q_0_new(3))-(q_0_new(2)-q_i(2,form(i)))*cos(q_0_new(3));
    delta_i(3,i) = q_0_new(3)-q_i(3,form(i));
    delta_i_expect(1,i)  =  -(q_0_new(1)-q_i_expect(1,i))*cos(q_0_new(3))-(q_0_new(2)-q_i_expect(2,i))*sin(q_0_new(3));
    delta_i_expect(2,i) = (q_0_new(1)-q_i_expect(1,i))*sin(q_0_new(3))-(q_0_new(2)-q_i_expect(2,i))*cos(q_0_new(3));
    delta_i_expect(3,i) = q_0_new(3)-q_i_expect(3,i);
    ex(i) = delta_i(1,i)-delta_i_expect(1,i);
    ey(i) = delta_i(2,i)-delta_i_expect(2,i);
    f1(i) = -nu_0(1)+delta_i_expect(2,i)*nu_0(3);
    f2(i) = -nu_0(2)-delta_i_expect(1,i)*nu_0(3);
    z1(i) = ex(i)*cos(delta_i(3,i)) - ey(i)*sin(delta_i(3,i));
    z2(i) = ex(i)*sin(delta_i(3,i)) + ey(i)*cos(delta_i(3,i));    
end

k1 = beta/alpha;
k2 = beta;
k4 = 2;
for i = 1:USV_number_form 
    nu_i_new(1,form(i)) = -k1*z1(i) - f1(i)*cos(delta_i(3,i))+f2(i)*sin(delta_i(3,i));
    nu_i_new(2,form(i)) = -k2*z2(i) - f1(i)*sin(delta_i(3,i))-f2(i)*cos(delta_i(3,i));
    nu_i_new(3,form(i)) = nu_0(3) + k4*sin(delta_i(3,i)); 
end
end
end


