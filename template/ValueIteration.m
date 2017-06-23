function [ J_opt, u_opt_ind ] = ValueIteration( P, G )
%VALUEITERATION Value iteration
%   Solve a stochastic shortest path problem by Value Iteration.
%
%   [J_opt, u_opt_ind] = ValueIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%
%       J_opt:
%       	A (K x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (K x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space.

% put your code here

%disp('running value iteration');

% constants
[num_states, num_inputs] = size(G);

% initializing policy
u_opt_ind = zeros(num_states, 1);
J_opt = zeros(num_states, 1);

% for each state
% calculate the best policy
% by adding G, then summing over average p * cost of p

tol = 1e-6;
converged = false;
%max_iterations = 100;

while ~converged
    
    % step 1: calculate best u for states
    for state_i = 1:num_states
        min_cost_so_far = realmax('double');
        
        for policy_l = 1:num_inputs
            policy_cost = G(state_i, policy_l);
            for state_j = 1:num_states
                policy_cost = policy_cost + P(state_i, state_j, policy_l) * J_opt(state_j);
            end
            if policy_cost < min_cost_so_far
                min_cost_so_far = policy_cost;
                u_opt_ind(state_i) = policy_l;
            end
        end
    end
    
    % step 2: calculate new J's
    old_J_opt = J_opt;
    
    for state_i = 1:num_states
        new_cost = G(state_i, u_opt_ind(state_i));
        
        for state_j = 1:num_states
            new_cost = new_cost + P(state_i, state_j, u_opt_ind(state_i)) * old_J_opt(state_j);
        end
        
        J_opt(state_i) = new_cost;
    end
    
    if norm(old_J_opt - J_opt) < tol
        converged = true;
    end
    
end

end