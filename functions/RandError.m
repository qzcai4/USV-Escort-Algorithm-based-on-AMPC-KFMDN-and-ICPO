function [Error] = RandError(mu)
mu1 = mu;  
mu2 = -mu;  
sigma1 = mu/3; 
sigma2 = sigma1; 

weight = randi([0,1]);
Error = weight*normrnd(mu1, sigma1)+(1-weight)*normrnd(mu2, sigma2);
 
end