clc;clear;close all;
time = 0;
singledis = 0;
tottime = 0;
totdis = 0;
newdis = 0;
newtotdis = 0;
for i = 1:1

map = imbinarize(imread('map5.bmp'));
imshow(map);
hold on;
q_start = [10;10];
q_goal = [490;490];
error = 10;
imax = 10000;
L = 10;
p_g = 0.3;
dim = 2;


tstart = tic;
SearchTree = [q_start;0;0];
RestTree = SearchTree;
  isReach = 0;
for iter = 1:imax
    if rand < p_g
        q_rand = transpose(rand(1,dim) .* size(map));  
        [q_near] = nearest(q_rand,SearchTree,dim);
        q_dir = (q_rand - q_near)/ norm(q_rand - q_near);
        q_new = round(q_near + L * q_dir);
        colli = collicheck(q_new,map);
        if colli == false
            SearchTree = [SearchTree,[q_new;q_near]];
            RestTree = [RestTree,[q_new;q_near]];
            plot([q_near(1) q_new(1)],[q_near(2) q_new(2)]);
         %   pause(0.1)
            if norm(q_goal - q_new) < error
                break;
            end
        end
    else %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if size(RestTree,2) ~= 0
            [q_old,RestTree] = nearest(q_goal,RestTree,dim);
            flag = true;
            while flag
                q_dir = (q_goal - q_old)/ norm(q_goal - q_old);
                q_new = round(q_old + L * q_dir);
                colli = collicheck(q_new,map);
                if colli == false
                    SearchTree = [SearchTree,[q_new;q_old]];                      
                   plot([q_old(1) q_new(1)],[q_old(2) q_new(2)]);
            %        pause(0.1)
                    q_old = q_new;
                    if norm(q_goal - q_new) < error
                        isReach = 1;
                        break;
                    end
                else
                    flag = false;
                end
            end
            if isReach == 1
                break;
            end
        end
    end
end



path = SearchTree(1:2,end);
temp = SearchTree(3:4,end);
for k = (size(SearchTree,2)-1):-1:1
    if SearchTree(1:2,k) == temp
        path = [temp,path];
        plot([temp(1) path(end,1)],[temp(2) path(end,2)],'color','r','LineWidth',2);
        temp = SearchTree(3:4,k); 
    end
end

tend = toc(tstart);

line(path(1,:),path(2,:),'color', 'r', 'LineWidth',2);
totdispath = 0;
for i = 1:size(path,2)-1
    dis = norm(path(:,i+1)-path(:,i));
    totdispath = totdispath + dis;
end
    
numIteration = 5*size(path,2);

%[newpath] = shorten(path, numIteration, error, L, map);


%tic;
newpath = path;
%function [path] = shorten(path, numIteration, error, L, map)
    for ker = 1:numIteration
        pathlength = size(newpath, 2);
        v = randi([2,pathlength - 1], [1,2]);
        v1 = min(v); v2 = max(v);
        if v1 == v2, continue;
        else
            startConfig = newpath(:,v1);
            endConfig = newpath(:,v2);
            [TF,tmpath] = onlytogoal(startConfig, endConfig, error, L, map);
            if TF == false, continue;
            else 
                newpath = [newpath(:,1:v1-1),tmpath,newpath(:,v2+1:end)];
            end
        end
    end
%toc;
%line(newpath(1,:),newpath(2,:),'color', 'g', 'LineWidth',2);
%end
totdisnewpath = 0;
for inew = 1:size(newpath,2)-1
    dis = norm(newpath(:,inew+1)-newpath(:,inew));
    totdisnewpath = totdisnewpath + dis;
end


%%disp(totdispath);
time = [time,tend];
tottime = tottime + tend;
singledis = [singledis,totdispath];
totdis = totdis + totdispath;
newdis = [newdis,totdisnewpath];
newtotdis = newtotdis + totdisnewpath;
end

avgtime = tottime/100;
stdtime = std(time);
avgdis = totdis/100;
stddis = std(singledis);
avgnewdis = newtotdis/100;
stdnewdis = std(newdis);

fprintf('avg time is %.3f, std time is %.2f\n', avgtime, stdtime);
fprintf('avg dis is %.2f, std dis is %.2f\n', avgdis, stddis);
fprintf('avg smooth dis is %.2f, std smooth dis is %.2f\n', avgnewdis, stdnewdis);


% function used to run straight line form start to end, 
% if colli check, do nothing 
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
