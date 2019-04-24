function [miu] = fuzz(x, L)
% Implementation of the fuzzification module of the variable 
% gain Fuzzy PI Controller
% 
%
if x < -L
    miu_p = 0;
    miu_n = 1;
elseif x > L
    miu_p = 1;
    miu_n = 0;
else
    miu_p = (x + L)/(2*L);
    miu_n = (-x + L)/(2*L);
end
        
miu = [miu_p; miu_n];
    
end