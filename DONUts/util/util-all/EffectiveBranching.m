%d = depth of solution (also equal to number of final moves)
%n = number of states, known from computation
%b_max = maximum number of branches per node (i.e. the number of modules, N)

%This is how you compute the effective branching factor
%Basically how many branches every node would have "on average" if the solution was
%recast as a BFS tree:
%n = b^0 + b^1 + b^2 + .... b^d; 
%No closed form solution, do the following instead:

% b_eff = compute_ebranch(203620,225,10); %Best b_eff is closer to 1, basically guessing the right solution every time.

function b_eff = compute_ebranch(n,d,N)
    %b_eff must be between 1 and N
    b_min=1; 
    b_max=N;
    MAX_ITER=100;
    EPSILON=0.0001;

   for i = 1:MAX_ITER
       b_mid=(b_min+b_max)/2;       
       nodes_mid = num_nodes_sum(d,b_mid); %Compute n based on guess of b (nodes_mid)
       if (nodes_mid <= n), b_min = b_mid;
       else, b_max = b_mid; end
       
       if (b_max-b_min) < EPSILON, break; end
   end
   b_eff = ((b_min+b_max)/2); %return actual effective branching factor
   disp('b_eff'); disp(b_eff);
end

function num_nodes=num_nodes_sum(d,b_guess)
    num_nodes=1;
    for i=1:d
        num_nodes = num_nodes + b_guess^i;
    end
end