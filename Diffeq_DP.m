clc; clear;

% Define parameters
% change parameters as necessary to solve other problems
N = 2; 
u = -1:0.02:1; 
x = 0:0.02:1.5;
del_t = 1;
b = 1;
a = 0;
lam = 2;
%initialize next x matrix
nextx = zeros(length(x),length(u));

% loop
for k = 1:N % perform algorithm for each state
    for i = 1:length(x) % at each state calculate difference equation for each control
        for j = 1:length(u) 
            % only calculate difference equation if next state is within
            % x constraints
            if ((1+a*del_t)*x(i) + b*del_t*u(j)) <= x(76) && ((1+a*del_t)*x(i) + b*del_t*u(j)) >= x(1)
                nextx(i,j) = (1+a*del_t)*x(i) + b*del_t*u(j); % difference equation
            end
            C_12(i,j) = nextx(i,j)^2 + lam*del_t*u(j)^2; % calculate cost
        end
        [minC_12(i),optval] = min(C_12(i,:)); % find minimum cost from each state to next state
        optcons(i) = u(optval); % find control path
    end
end

