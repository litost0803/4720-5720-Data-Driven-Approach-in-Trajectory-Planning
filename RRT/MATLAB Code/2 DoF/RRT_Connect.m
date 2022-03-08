clc;clear;close all;

time = 0;
singledis = 0;
tottime = 0;
totdis = 0;
newdis = 0;
newtotdis = 0;

for i=1:1
%%%%% parameters
map=imbinarize(imread('map3.bmp')); % input map read from a bmp file. for new maps write the file name here
source=[10 10]; % source position in Y, X format
goal=[490 490]; % goal position in Y, X format
stepsize=10; 	% size of each step of the RRT
disTh=10; 		% nodes closer than this threshold are taken as almost the same
maxFailedAttempts = 10000;
display=true; 	% display of RRT

tstart = tic;

if ~feasiblePoint(source,map)
    error('source lies on an obstacle or outside map'); 
end
if ~feasiblePoint(goal,map)
    error('goal lies on an obstacle or outside map'); 
end
if display
    imshow(map);
    rectangle('position',[1 1 size(map)-1],'edgecolor','k'); 
end

RRTree1 = double([source -1]); % First RRT rooted at the source, representation node and parent index
RRTree2 = double([goal -1]);   % Second RRT rooted at the goal, representation node and parent index
counter=0;
tree1ExpansionFail = false; % sets to true if expansion after set number of attempts fails
tree2ExpansionFail = false; % sets to true if expansion after set number of attempts fails

while ~tree1ExpansionFail || ~tree2ExpansionFail  % loop to grow RRTs
    if ~tree1ExpansionFail 
        [RRTree1,pathFound,tree1ExpansionFail] = rrtExtend(RRTree1,RRTree2,goal,stepsize,maxFailedAttempts,disTh,map); % RRT 1 expands from source towards goal
        if ~tree1ExpansionFail && isempty(pathFound) && display
            line([RRTree1(end,2);RRTree1(RRTree1(end,3),2)],[RRTree1(end,1);RRTree1(RRTree1(end,3),1)],'color','b');
       %      counter=counter+1;M(counter)=getframe;
%             pause(0.5);
             
        end
    end
    if ~tree2ExpansionFail 
        [RRTree2,pathFound,tree2ExpansionFail] = rrtExtend(RRTree2,RRTree1,source,stepsize,maxFailedAttempts,disTh,map); % RRT 2 expands from goal towards source
        if ~isempty(pathFound), pathFound(3:4)=pathFound(4:-1:3); end % path found
        if ~tree2ExpansionFail && isempty(pathFound) && display
            line([RRTree2(end,2);RRTree2(RRTree2(end,3),2)],[RRTree2(end,1);RRTree2(RRTree2(end,3),1)],'color','r');
      %      counter=counter+1;M(counter)=getframe;
%             pause(0.5);
        end
    end
    if ~isempty(pathFound) % path found
         if display
            line([RRTree1(pathFound(1,3),2);pathFound(1,2);RRTree2(pathFound(1,4),2)],[RRTree1(pathFound(1,3),1);pathFound(1,1);RRTree2(pathFound(1,4),1)],'color','green');
        %    counter=counter+1;M(counter)=getframe;
           % pause(0.5);
        end
        path=[pathFound(1,1:2)]; % compute path
        prev=pathFound(1,3);     % add nodes from RRT 1 first
        while prev > 0
            path=[RRTree1(prev,1:2);path];
            prev=RRTree1(prev,3);
        end
        prev=pathFound(1,4); % then add nodes from RRT 2
        while prev > 0
            path=[path;RRTree2(prev,1:2)];
            prev=RRTree2(prev,3);
        end
        break;
    end
end
tend = toc(tstart);
% if display 
%     disp('click/press any key');
%     waitforbuttonpress; 
% end

if size(pathFound,1)<=0, error('no path found. maximum attempts reached'); end
pathLength=0;
 for i=1:length(path)-1, pathLength=pathLength+distanceCost(path(i,1:2),path(i+1,1:2)); end
% fprintf('processing time=%d \nPath Length=%d \n\n', pathLength); 
%imshow(map);
rectangle('position',[1 1 size(map)-1],'edgecolor','k');
line(path(:,2),path(:,1));

numIteration = 5*size(path,1);
newpath = [];
newpath(2,:) = path(:,1);
newpath(1,:) = path(:,2);
%function [path] = shorten(path, numIteration, error, L, map)
    for ker = 1:numIteration
        pathlength = size(newpath, 2);
        v = randi([2,pathlength - 1], [1,2]);
        v1 = min(v); v2 = max(v);
        if v1 == v2, continue;
        else
            startConfig = newpath(:,v1);
            endConfig = newpath(:,v2);
            [TF,tmpath] = onlytogoal(startConfig, endConfig, disTh, stepsize, map);
            if TF == false, continue;
            else 
                newpath = [newpath(:,1:v1-1),tmpath,newpath(:,v2+1:end)];
            end
        end
    end
%toc;
line(newpath(1,:),newpath(2,:),'color', 'g', 'LineWidth',2);
%end
totdisnewpath = 0;
for inew = 1:size(newpath,2)-1
    dis = norm(newpath(:,inew+1)-newpath(:,inew));
    totdisnewpath = totdisnewpath + dis;
end


time = [time,tend];
tottime = tottime + tend;
singledis = [singledis,pathLength];
totdis = totdis + pathLength;
newdis = [newdis,totdisnewpath];
newtotdis = newtotdis + totdisnewpath;
end


avgtime = tottime/1;
stdtime = std(time);
avgdis = totdis/1;
stddis = std(singledis);
avgnewdis = newtotdis/1;
stdnewdis = std(newdis);

fprintf('avg time is %.2f, std time is %.2f\n', avgtime, stdtime);
fprintf('avg dis is %.2f, std dis is %.2f\n', avgdis, stddis);
fprintf('avg smooth dis is %.2f, std smooth dis is %.2f\n', avgnewdis, stdnewdis);



%% distanceCost.m
function h=distanceCost(a,b)
	h = sqrt(sum((a-b).^2, 2));
end	
	
%% checkPath.m	
function feasible=checkPath(n,newPos,map)
feasible=true;
dir=atan2(newPos(1)-n(1),newPos(2)-n(2));
for r=0:0.5:sqrt(sum((n-newPos).^2))
    posCheck=n+r.*[sin(dir) cos(dir)];
    if ~(feasiblePoint(ceil(posCheck),map) && feasiblePoint(floor(posCheck),map) && ... 
            feasiblePoint([ceil(posCheck(1)) floor(posCheck(2))],map) && feasiblePoint([floor(posCheck(1)) ceil(posCheck(2))],map))
        feasible=false;break;
    end
    if ~feasiblePoint(newPos,map), feasible=false; end
end
end


%% feasiblePoint.m
function feasible=feasiblePoint(point,map)
feasible=true;
% check if collission-free spot and inside maps
if ~(point(1)>=1 && point(1)<=size(map,1) && point(2)>=1 && point(2)<=size(map,2) && map(point(1),point(2))==1)
    feasible=false;
end
end

%% rrtExtend
function [RRTree1,pathFound,extendFail] = rrtExtend(RRTree1,RRTree2,goal,stepsize,maxFailedAttempts,disTh,map)
pathFound=[]; %if path found, returns new node connecting the two trees, index of the nodes in the two trees connected
failedAttempts=0;
while failedAttempts <= maxFailedAttempts
    if rand < 0.5 
        sample = rand(1,2) .* size(map); % random sample
    else
        sample = goal; 	% sample taken as goal to bias tree generation to goal
    end
	
    [A, I] = min( distanceCost(RRTree1(:,1:2),sample) ,[],1); % find the minimum value of each column
    closestNode = RRTree1(I(1),:);
	
	%% moving from qnearest an incremental distance in the direction of qrand
    theta = atan2((sample(1)-closestNode(1)),(sample(2)-closestNode(2)));  % direction to extend sample to produce new node
    newPoint = double(int32(closestNode(1:2) + stepsize * [sin(theta)  cos(theta)]));
    if ~checkPath(closestNode(1:2), newPoint, map) % if extension of closest node in tree to the new point is feasible
        failedAttempts = failedAttempts + 1;
        continue;
    end
	
    [A, I2] = min( distanceCost(RRTree2(:,1:2),newPoint) ,[],1); % find closest in the second tree
    if distanceCost(RRTree2(I2(1),1:2),newPoint) < disTh        % if both trees are connected
        pathFound=[newPoint I(1) I2(1)];extendFail=false;
        break; 
    end 
    [A, I3] = min( distanceCost(RRTree1(:,1:2),newPoint) ,[],1); % check if new node is not already pre-existing in the tree
    if distanceCost(newPoint,RRTree1(I3(1),1:2)) < disTh
        failedAttempts=failedAttempts+1;
        continue; 
    end 
    RRTree1 = [RRTree1;newPoint I(1)];extendFail=false;break; % add node
end
end



function [TF,tmpath] = onlytogoal(startConfig, endConfig, error, L, map)
    tmpath = startConfig;
    TF = true;
    tmax= 100;
    for t = 1:tmax
        [tqnear] = nearest(endConfig,tmpath,2);
        tqdir = (endConfig - tqnear)/norm(endConfig - tqnear);
        tqnew = round(tqnear + L * tqdir);
        colli = collicheck(tqnew,map);
        if colli == false
            tmpath = [tmpath,tqnew];
            if norm(endConfig - tqnew) < error
                break;
            end
        else
            TF = false;
            break;
        end
    end
end



function colli = collicheck(q,map)
colli = true;
if q(1) > 0 && q(1) < size(map,1) && q(2) >0 && q(2) < size(map,1)
    if map(q(2,1),q(1,1)) == 1
        colli = false;
    end
end
end

function [q,Tree] = nearest(qin,Tree,dim)
    tmp = Tree(1:dim,1);
    dis = norm(tmp - qin);
    k = 1;
    for i = 1:size(Tree,2)
        newtmp = Tree(1:dim,i);
        newdis = norm(newtmp - qin);
        if newdis < dis
            k = i;
        tmp = newtmp;
        dis = newdis;
        end
    end
    q = tmp; 
    [n,m] = find(Tree(1:dim,:) == q);
    Tree(:,k) = [];
end

