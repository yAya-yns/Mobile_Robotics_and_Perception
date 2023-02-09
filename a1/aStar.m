function spath = aStar(G, startIdx, finishIdx, milestones_map, hWeight, maxIter)

if ~exist('maxIter','var')
     % maxIter does not exist, so default it to something
      maxIter = 10000;
 end

% aStar Init:
G.Nodes.localCost = Inf(numnodes(G), 1);
G.Nodes.globalCost = Inf(numnodes(G), 1);
G.Nodes.parent = NaN(numnodes(G), 1);
G.Nodes.visited = zeros(numnodes(G), 1);

startIdx_nid = findnode(G,startIdx);
startIdx_globalCost = heuristics(startIdx, finishIdx, milestones_map);

G.Nodes.localCost(startIdx_nid) = 0;
G.Nodes.globalCost(startIdx_nid) = startIdx_globalCost;

searchQ= [startIdx, startIdx_globalCost];

iter = 1;
while  iter <= maxIter && ~isempty(searchQ(:, 1))
    [searchQ, G] = update(G, finishIdx, searchQ, milestones_map, hWeight);
%     disp(searchQ);  
    iter = iter + 1;
end

if iter > maxIter
    disp('maxIter exceeded for aStar, quitting');
    spath = [];
    return
end    

spath_reversed = [finishIdx];
curIdx = spath_reversed(1);
parentIdx = NaN; % initializing to a non meaningful value
while parentIdx ~= startIdx
    try
        parentIdx = G.Nodes.parent(curIdx);
        spath_reversed(end+1) = parentIdx;
        curIdx = parentIdx;
    catch
        disp('ERROR: curIdx is NaN, potentially no solution exists');
        disp('Check if you have enough branch and points');
        spath = []
        return
    end
%     disp(spath_reversed);
end
spath = flip(spath_reversed);


function [searchQ, G] = update(G, finishIdx, searchQ, milestones_map, hWeight)
cur_nid = searchQ(1, 1);  % getting idx of the first element from searchQ
cur_neighbors = neighbors(G,cur_nid);
eid_cur_out = findedge(G, cur_nid*ones(1, length(cur_neighbors)), cur_neighbors');
weight_cur_out =  G.Edges.Weight(eid_cur_out);
for i = 1:length(cur_neighbors)
    if G.Nodes.localCost(cur_neighbors(i)) > (G.Nodes.localCost(cur_nid) + weight_cur_out(i))
        G.Nodes.localCost(cur_neighbors(i)) = G.Nodes.localCost(cur_nid) + weight_cur_out(i);
        G.Nodes.globalCost(cur_neighbors(i)) = G.Nodes.localCost(cur_neighbors(i)) + hWeight * heuristics(cur_neighbors(i), finishIdx, milestones_map);
        G.Nodes.parent(cur_neighbors(i)) = cur_nid;
    end
    
    if G.Nodes.visited(cur_neighbors(i)) ~= 1
        searchQ(end + 1, :) = [cur_neighbors(i), G.Nodes.globalCost(cur_neighbors(i))];
        G.Nodes.visited(cur_neighbors(i)) = 1;
    end
end
searchQ = searchQ(2:end, :); % removing the node visited just now
searchQ = sortSearchQ(searchQ);


function hVal = heuristics(ptIdx, finishIdx, milestones_map)
hVal = norm(milestones_map(ptIdx, 2:3)-milestones_map(finishIdx, 2:3));


function sortedSearchQ = sortSearchQ(searchQ)
sortedSearchQ = sortrows(searchQ, 2); 






