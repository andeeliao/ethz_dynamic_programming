function G = ComputeStageCosts( stateSpace, controlSpace, map, gate, mansion, cameras )
%COMPUTESTAGECOSTS Compute stage costs.
% 	Compute the stage costs for all states in the state space for all
%   control inputs.
%
%   G = ComputeStageCosts(stateSpace, controlSpace, map, gate, mansion,
%   cameras) computes the stage costs for all states in the state space
%   for all control inputs.
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
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.

% Transform map into m-n coordinate system.
map = fliplr(flipud(map)');

% Get dimensions.
L = length(controlSpace);
K = length(stateSpace);

% Create G.
G = Inf(K,L);

global detected_additional_time_steps
global pool_num_time_steps

% Find state index of gate.
[~, gate_ind] = ismember(gate, stateSpace, 'rows');

%Basic code:
for i=1:K
    
    for u=1:L
        
        pictureProb = 0;
        cellCost = 1;
        
        switch controlSpace(u)
            case 'n'
                newState = stateSpace(i,:) + [0,1];
            case 'w'
                newState = stateSpace(i,:) + [-1,0];
            case 's'
                newState = stateSpace(i,:) + [0,-1];
            case 'e'
                newState = stateSpace(i,:) + [1,0];
            case 'p'
                newState = stateSpace(i,:);
                pictureProb = computePictureProbability(newState, mansion, map);
        end
        
        % Account for infeasible cells.
        % If new state is not in state space, it is infeasible -> cost
        % stays infinite.
        if ismember(newState, stateSpace, 'rows')
            [~, new_state_ind] = ismember(newState, stateSpace, 'rows');
            detectProb = computeTotalDetectionProbability(newState, map, cameras);
            % Negative cells belong to pools or ponds
            % Higher cost associated with pools or ponds if we are not
            % already in them.
            if (map(newState(1), newState(2)) < 0 && map(stateSpace(i,1), stateSpace(i,2)) >= 0)
                cellCost = pool_num_time_steps;
            end
            % The cost of moving ot the new cell
            % (4 for pond or pool, 1 otherwise)
            movementCost = cellCost;
            % The cost of getting detected
            % Detection probability gets multiplied by the cost factor of the
            % cell
            % (If in a pond or pool, 4 attempts to get detected)
            detectProb = 1-(1-detectProb)^cellCost;
            % Total cost = fail to take a picture AND get detected,
            % multiplied by cost of returning to gate
            detectionCost = (1-pictureProb)*detectProb*detected_additional_time_steps;
            G(i,u) = movementCost + detectionCost;
        end
        
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