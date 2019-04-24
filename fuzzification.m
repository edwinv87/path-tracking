function [memn] = fuzzification(nn, n) 
%Fuzzification based on triangle membership functions
%Coded by Edwin Vans
%
%
%
mfn = length(nn);
memn = zeros(1, mfn);
if n <= nn(1)                %the leftmost case 
    memn(1) = 1;
end

if n >= nn(mfn)              %the rightmost case
    memn(mfn) = 1;
end

for k = 1:(mfn - 1)
    if ((n > nn(k)) && (n <= nn(k+1)))
        Fn = (n - nn(k))/(nn(k+1) - nn(k));     %this can be changed to get other membership functions
        memn(k+1) = Fn;
        memn(k) = 1 - Fn;
    end
end
end