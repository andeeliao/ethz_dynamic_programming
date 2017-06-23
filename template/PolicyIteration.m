function [ J_opt, u_opt_ind ] = PolicyIteration( P, G )
%POLICYITERATION Value iteration
%   Solve a stochastic shortest path problem by Policy Iteration.
%
%   [J_opt, u_opt_ind] = PolicyIteration(P, G) computes the optimal cost and
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

% constants
[num_states, num_inputs] = size(G);

% initializing policy
u_opt_ind = ones(num_states, 1);
J_opt = ones(num_states, 1);
%J_opt = repmat(realmax('double'),1, num_states);

converged = false;
%max_iterations = 100;
disp('running policy iteration 1')

while ~converged
   
    % step 1: solve for cost associated with policy
    for state_i = 1:num_states
       J_opt(state_i) = G(state_i, u_opt_ind(state_i));
       
       for state_j = 1:num_states
           additional_cost = P(state_i, state_j, u_opt_ind(state_i)) * J_opt(state_j);
           if isnan(additional_cost)
               additional_cost = 0;
           end
           J_opt(state_i) = J_opt(state_i) + additional_cost;
       end
    end
    
    % step 2: improve policy 
    disp('running policy iteration checkpoint 1')
    
    old_u_opt_ind = u_opt_ind;
    
    for state_i = 1:num_states
       min_cost_so_far = J_opt(state_i);
       for policy_q = 1:num_inputs
           policy_cost = G(state_i, policy_q);
           for state_j = 1:num_states
               additional_cost = P(state_i, state_j, policy_q) * J_opt(state_j);
               if isnan(additional_cost)
                   additional_cost = 0;
               end
               policy_cost = policy_cost + additional_cost;
           end
           if policy_cost < min_cost_so_far
               min_cost_so_far = policy_cost;
               u_opt_ind(state_i) = policy_q;
           end
       end
    end
    
    if isequal(old_u_opt_ind, u_opt_ind)
        converged = true;
    end
    
    disp('running policy iteration 2');
    
end

end

