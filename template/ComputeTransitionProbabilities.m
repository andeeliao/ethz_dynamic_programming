function P = ComputeTransitionProbabilities( stateSpace, controlSpace, map, gate, mansion, cameras )
%COMPUTETRANSITIONPROBABILITIES Compute transition probabilities.
% 	Compute the transition probabilities between all states in the state
%   space for all control inputs.
%
%   P = ComputeTransitionProbabilities(stateSpace, controlSpace,
%   map, gate, mansion, cameras) computes the transition probabilities
%   between all states in the state space for all control inputs.
%
%   Input arguments:
%
%       stateSpace:
%           A (K x 2)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       controlSpace:
%           A (L x 1)-matrix, where the l-th row represents the l-th
%           element of the control space.
%
%       map:
%           A (M x N)-matrix describing the terrain of the estate map.
%           Positive values indicate cells that are inaccessible (e.g.
%           trees, bushes or the mansion) and negative values indicate
%           ponds or pools.
%
%   	gate:
%          	A (1 x 2)-matrix describing the position of the gate.
%
%    	mansion:
%          	A (F x 2)-matrix indicating the position of the cells of the
%           mansion.
%
%    	cameras:
%          	A (H x 3)-matrix indicating the positions and quality of the 
%           cameras.
%
%   Output arguments:
%
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.

% Transform map into m-n coordinate system.
map = fliplr(flipud(map)');

% Get dimensions.
L = length(controlSpace);
K = length(stateSpace);

% Create P.
P = zeros(K, K, L);

% Find state index of gate.
[~, gate_ind] = ismember(gate, stateSpace, 'rows');

% Loop through all states.
for i = 1:K
   
    % Loop through all controls.
    for j = 1:L
           
        pictureProb = 0;
        
        switch controlSpace(j)
            
            case 'n'
                newState = stateSpace(i,:) + [0,1];
            case 's'
                newState = stateSpace(i,:) + [0,-1];
            case 'e'
                newState = stateSpace(i,:) + [1,0];
            case 'w'
                newState = stateSpace(i,:) + [-1,0];
            case 'p'
                newState = stateSpace(i,:);
                pictureProb = computePictureProbability(newState, mansion, map);
        end
        
        % Account for infeasible cells.
        if ~ismember(newState, stateSpace, 'rows')
            newState = stateSpace(i,:);
            new_state_ind = i;
        else
           [~, new_state_ind] = ismember(newState, stateSpace, 'rows'); 
        end
        
        % Probability of going to next state: don't get detected AND fail
        % at picture.
        detectProb = computeTotalDetectionProbability(newState, map, cameras);
        P(i, new_state_ind, j) =  (1 - detectProb) * (1 - pictureProb);
        % Probability of going to gate: get detected AND fail at picture.
        % OR case where action already leads you to gate.
        P(i, gate_ind, j) = detectProb * (1 - pictureProb) + P(i, gate_ind, j);
        
    end
    
end

end


% Computes the probability of being detected by all cameras given current
% paparazzi state.
function detectProb = computeTotalDetectionProbability(state, map, cameras)

probs = zeros(size(cameras,1), 1);

% Loop through all cameras and calculate probabilities for each.
for i = 1:size(cameras, 1)
    
    probs(i) = computeDetectionProbability(state, cameras(i, :), map);
    
end

% Sum probabilities.
% Total prob of being detected is the negation of not being detected by all
% cameras (AND).
% detectProb = ~(~prob1 * ~prob2 + ... + ~probH)
notDetectProb = 1;

for i = 1:size(probs, 1)
    notDetectProb = notDetectProb*(1-probs(i));
end

detectProb = 1 - notDetectProb;

end

% Compute detection probability for a particular camera given current
% paparazzi state.
function cameraDetectProb = computeDetectionProbability(currentState, cameraState, map)

% Walking into a camera - immediate detection?
% TODO: - Ask TAs about this.
if currentState(1:2) == cameraState(1:2)
    cameraDetectProb = 1;
    return;
end

% Set defaults.
cameraDetectProb = 0;
inHorizontalFoV = 1;
inVerticalFoV = 1;

% Check horizontal field of view.
if currentState(2) == cameraState(2)
    
    if currentState(1) < cameraState(1)
        horizontalCells = currentState(1):cameraState(1)-1;
    else
        horizontalCells = cameraState(1)+1:currentState(1);
    end
    
    for i = horizontalCells
        
        % Path is blocked.
        if map(i,currentState(2)) > 0
            inHorizontalFoV = 0;
            break;
        end
        
    end
    
else
    
    inHorizontalFoV = 0;
    
end

% If visible, compute probability.
if (inHorizontalFoV)
    cameraDetectProb = cameraState(3) / pdist([currentState; cameraState(1:2)]);
    return;
end

% Check vertical field of view.
if currentState(1) == cameraState(1)
    
    if currentState(2) < cameraState(2)
        verticalCells = currentState(2):cameraState(2)-1;
    else
        verticalCells = cameraState(2)+1:currentState(2);
    end
    
    for i = verticalCells
        
        % Path is blocked.
        if map(currentState(1),i) > 0
            inVerticalFoV = 0;
            break;
        end
        
    end
    
else
    
    inVerticalFoV = 0;
    
end

% If visible, compute probability.
if (inVerticalFoV)
    cameraDetectProb = cameraState(3) / pdist([currentState; cameraState(1:2)]);
    return;
end

end

% Compute probability of taking a successful picture of celebrity given current
% paparazzi state.
function pictureProb = computePictureProbability(currentState, mansionStates, ...
    map)

global p_c
global gamma_p

% Set defaults.
pictureProb = p_c;

% Check horizontal field of view.
hor_ind = find(currentState(1) == mansionStates(:,1));

if ~isempty(hor_ind)
    
    for m = 1:length(hor_ind)
        
        inHorizontalFoV = 1;
    
        if currentState(2) < mansionStates(hor_ind(m),2)
            horizontalCells = currentState(2):mansionStates(hor_ind(m),2)-1;
        else
            horizontalCells = mansionStates(hor_ind(m),2)+1:currentState(2);
        end

        for i = horizontalCells

            % Path is blocked.
            if map(currentState(1),i) > 0
                inHorizontalFoV = 0;
                break;
            end

        end
        
        if (inHorizontalFoV)
            break;
        end
        
    end
    
else
    
    inHorizontalFoV = 0;
    
end

% If visible, compute probability.
if (inHorizontalFoV)
    
    pictureProb = gamma_p / pdist([currentState; mansionStates(hor_ind(m),:)]);
    % Account for celebrity walking into picture.
    pictureProb = max(pictureProb, p_c);
    return;

end

% Check horizontal field of view.
vert_ind = find(currentState(2) == mansionStates(:,2));

% Check vertical field of view.
if ~isempty(vert_ind)
    
    for m = 1:length(vert_ind)
        
        inVerticalFoV = 1;
        
        if currentState(1) < mansionStates(vert_ind(m),1)
            verticalCells = currentState(1):mansionStates(vert_ind(m),1)-1;
        else
            verticalCells = mansionStates(vert_ind(m),1)+1:currentState(1);
        end
        
        for i = verticalCells
            
            % Path is blocked.
            if map(i,currentState(2)) > 0
                inVerticalFoV = 0;
                break;
            end
            
        end
        
        if (inVerticalFoV)
            break;
        end
        
    end
    
else
    
    inVerticalFoV = 0;
    
end

% If visible, compute probability.
if (inVerticalFoV)
    
    pictureProb = gamma_p / pdist([currentState; mansionStates(vert_ind(m),:)]);
    % Account for celebrity walking into picture.
    pictureProb = max(pictureProb, p_c);
    return;
    
end

end