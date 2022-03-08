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
p_g = 0.95;
dim = 2;
collichecktime = 0;

tstart = tic;
ST1 = [q_start;0;0];
ST2 = [q_goal;0;0];
RT1 = [q_start;0;0];
RT2 = [q_goal;0;0];
q_update1 = q_goal;
q_update2 = q_start;

  isReach = 0;
for iter = 1:imax
    if size(ST1,2) >= size(ST2,2)
        [ST2, RT2, ST1, isReach, collichecktime, q2end, q1end, q_update1] = ExtendTree(ST2, RT2, ST1, q_start, p_g, map, error, dim, L,collichecktime);
        if isReach == 1, break; end
    else
        [ST1, RT1, ST2, isReach, collichecktime, q1end, q2end, q_update2 ] = ExtendTree(ST1, RT1, ST2, q_goal, p_g, map, error, dim, L,collichecktime);
        if isReach == 1, break; end
    end
end


path1 = q1end;
half1path = q1end;
for k = (size(ST1,2)):-1:2
    if ST1(1:2,k) == path1
        temp1 = ST1(3:4,k); 
        half1path = [temp1,half1path];
         plot([temp1(1) path1(1)],[temp1(2) path1(2)],'Color','r','LineWidth',2);
%         pause(0.2);
        path1 = temp1;
    end
end

path2 = q2end;
half2path = q2end;
for k = (size(ST2,2)):-1:2
    if ST2(1:2,k) == path2
        temp2 = ST2(3:4,k); 
        half2path = [temp2,half2path];
         plot([temp2(1) path2(1)],[temp2(2) path2(2)],'Color','r','LineWidth',2);
%         pause(0.2);
        path2 = temp2;
    end
end

path = [half1path,fliplr(half2path)];


tend = toc(tstart);

line(path(1,:),path(2,:),'color', 'r', 'LineWidth',2);
totdispath = 0;
for ii = 1:size(path,2)-1
    dis = norm(path(:,ii+1)-path(:,ii));
    totdispath = totdispath + dis;
end
    
numIteration = 5*size(path,2);

%[newpath] = shorten(path, numIteration, error, L, map);



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
            [TF,tmpath,collichecktime] = onlytogoal(startConfig, endConfig, error, L, map,collichecktime);
            if TF == false, continue;
            else 
                newpath = [newpath(:,1:v1-1),tmpath,newpath(:,v2+1:end)];
            end
        end
    end
line(newpath(1,:),newpath(2,:),'color', 'g', 'LineWidth',2);
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

avgtime = tottime/1;
stdtime = std(time);
avgdis = totdis/1;
stddis = std(singledis);
avgnewdis = newtotdis/1;
stdnewdis = std(newdis);

fprintf('avg time is %.3f, std time is %.2f\n', avgtime, stdtime);
fprintf('avg dis is %.2f, std dis is %.2f\n', avgdis, stddis);
fprintf('avg smooth dis is %.2f, std smooth dis is %.2f\n', avgnewdis, stdnewdis);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [EST, ERT, OST, isReach, collichecktime, q1end, q2end, q_update] = ExtendTree(EST, ERT, OST, q, p_g, map, error, dim, L,collichecktime)
    isReach = 0;
    q1end = []; q2end = [];
    q_update = q;
    if rand < p_g
        q_rand = transpose(rand(1,dim) .* size(map));  
        [q_near] = nearest(q_rand,EST,dim);
        q_dir = (q_rand - q_near)/ norm(q_rand - q_near);
        q_new = round(q_near + L * q_dir);
        [colli,collichecktime] = collicheck(q_new,map,collichecktime);
        if colli == false
            EST = [EST,[q_new;q_near]];
            ERT = [ERT,[q_new;q_near]];
            plot([q_near(1) q_new(1)],[q_near(2) q_new(2)],'color','#0072BD','LineWidth',2);
%             pause(0.2)
            q_update = q_new;
            [q_othernear, dist] = nearest(q_new, OST, dim);
            if dist <= error
                plot([q_othernear(1) q_new(1)],[q_othernear(2) q_new(2)],'color','green','LineWidth',2);
     %           pause(0.2);
                isReach = 1; q1end = q_new; q2end = q_othernear;
                return;
            else
                [OST, isReach,collichecktime, qreturn] = TowardGoal(OST, q_othernear, q_new, error, map, L,collichecktime);
                if isReach == 1
                    q1end = q_new; q2end = qreturn;
                    return; 
                end
            end
        end
    else
        if size(ERT,2) ~= 0
            [q_old,dist,ERT] = nearest(q,ERT,dim);
            if dist <= error
                 plot([q_old(1) q(1)],[q_old(2) q(2)],'color','green','LineWidth',3);
%                 pause(0.2);
                isReach = 1;
                return;
            end
            q_update = q_old;
            [EST, isReach,collichecktime, qreturn] = TowardGoal(EST, q_old, q, error, map, L,collichecktime);
            if isReach == 1
                q1end = qreturn; q2end = q;
                return; 
            end
        end

    end
end


function [Tree, isReach,collichecktime,qreturn] = TowardGoal(Tree, q_start, q_goal, error, map, L,collichecktime)
    flag = true;
    isReach = 0;
    qreturn = [];
    while flag
        q_dir = (q_goal - q_start)/ norm(q_goal - q_start);
        q_new = round(q_start + L * q_dir);
        [colli,collichecktime] = collicheck(q_new,map,collichecktime);
        if colli == false
            Tree = [Tree,[q_new;q_start]];                      
             plot([q_start(1) q_new(1)],[q_start(2) q_new(2)],'color','#7E2F8E','LineWidth',2);
%             pause(0.2);
    %       pause(0.1)
            q_start = q_new;
            if norm(q_goal - q_new) < error
                 plot([q_goal(1) q_new(1)],[q_goal(2) q_new(2)],'color','green','LineWidth',2);
%                 pause(0.2);
                qreturn = q_new;
                isReach = 1;
                break;
            end
        else
            flag = false;
        end
    end
    if isReach == 1
        return;
    end
end


%end
function [TF,tmpath,collichecktime] = onlytogoal(startConfig, endConfig, error, L, map,collichecktime)
    tmpath = startConfig;
    TF = true;
    tmax= 100;
    for t = 1:tmax
        [tqnear] = nearest(endConfig,tmpath,2);
        tqdir = (endConfig - tqnear)/norm(endConfig - tqnear);
        tqnew = round(tqnear + L * tqdir);
        [colli,collichecktime] = collicheck(tqnew,map,collichecktime);
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



function [colli,collichecktime] = collicheck(q,map,collichecktime)
colli = true;
if q(1) > 0 && q(1) < size(map,1) && q(2) >0 && q(2) < size(map,1)
    if map(q(2,1),q(1,1)) == 1
        colli = false;
    end
end
collichecktime = collichecktime + 1;
end


function [q,dist,Tree] = nearest(qin,Tree,dim)
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
    dist = newdis;
    [n,m] = find(Tree(1:dim,:) == q);
    Tree(:,k) = [];
end
