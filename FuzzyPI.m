function u = FuzzyPI(x1, x2, a)
% Implementation of the variable gain Fuzzy PI controller
% p = [k, a], where
%
%
%
L = 1;          % Design Parameter (Range for the input)
M = 2;          % No. of inputs
N = 2^M;        % No. of fuzzy rules
k = [1, 0, 0, 1];
% a = [0.5, 0.5];
a = a';
% Fuzzification
miu1 = fuzz(x1, L);
miu2 = fuzz(x2, L);

% Inference 
miu = zeros(1, N);
miu(1) = miu1(1)*miu2(1);
miu(2) = miu1(1)*miu2(2);
miu(3) = miu1(2)*miu2(1);
miu(4) = miu1(2)*miu2(2);

% Defuzzification
F = sum(miu.*k);
u = F*sum(a.*[x1, x2]);
% u = F;
end
