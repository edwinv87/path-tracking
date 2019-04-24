function [z] = FuzzyDriftControl(x, y) 
%FUZZY CONTROLLER 336

%PARAMETER BASE------------------------------------------------------------
%For Input X
xx = [-1, 0, 1];

%For Input Y
yy = [0, 0.5, 1];

%Output Z - Singletons
sl = [0.2, 0.5, 1];                               %singleton levels

%Fuzzy Rule Base
% column - Steering(Input X)
% row - Bend (Input Y)
FRB = [sl(3), sl(3), sl(2);
       sl(3), sl(2), sl(1);
       sl(2), sl(1), sl(1)];


%FUZZIFICATION-------------------------------------------------------------
memx = fuzzification(xx, x);
memy = fuzzification(yy, y);

%FUZZY INFERENCE-----------------------------------------------------------
%Activation degrees calculation using product operation
actv = memy' * memx;
% disp(actv);

%DEFUZZIFICATION-----------------------------------------------------------
%Using weighted average method
sum1 = sum(sum(actv.*FRB));
sum2 = sum(sum(actv));
z = sum1/sum2;
%disp('Fuzzy Control Output: ');
%disp(z);
end