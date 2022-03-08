clc;clear;close all;
map = imbinarize(imread('map5.bmp'));
imshow(map);
hold on;
q_start = [10;10];
q_goal = [490;490];
error = 5;
imax = 10000;
L = 5;
p_g = 0.7;
dim = 2;


tic;
SearchTree = [q_start;0;0;0];
RestTree = SearchTree;
  isReach = 0;
for iter = 1:imax
    if rand < p_g
        q_rand = transpose(rand(1,dim) .* size(map));  
        [q_near,index] = nearest(q_rand,SearchTree,dim);
        q_dir = (q_rand - q_near)/ norm(q_rand - q_near);
        q_new = round(q_near + L * q_dir);
        colli = collicheck(q_new,map);
        if colli == false
            SearchTree = [SearchTree,[q_new;index+1;q_near]];
            RestTree = [RestTree,[q_new;index+1;q_near]];
            plot([q_near(1) q_new(1)],[q_near(2) q_new(2)]);
         %   pause(0.1)
            if norm(q_goal - q_new) < error
                break;
            end
        end
    else %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if size(RestTree,2) ~= 0
            [q_old,index,RestTree] = nearest(q_goal,RestTree,dim);
            flag = true;
            while flag
                q_dir = (q_goal - q_old)/ norm(q_goal - q_old);
                q_new = round(q_old + L * q_dir);
                colli = collicheck(q_new,map);
                if colli == false
                    index = index +1;
                    SearchTree = [SearchTree,[q_new;index;q_old]];                      
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
toc;


path = SearchTree(1:2,end);
temp = SearchTree(4:5,end);
for k = (size(SearchTree,2)-1):-1:1
    if SearchTree(1:2,k) == temp
        path = [temp,path];
      %  plot([temp(1) path(end,1)],[temp(2) path(end,2)],'edgecolor','r','LineWidth ',0.7);
        temp = SearchTree(4:5,k); 
    end
end

line(path(1,:),path(2,:),'color', 'r', 'LineWidth',2);



function colli = collicheck(q,map)
colli = true;
if q(1) > 0 && q(1) < size(map,1) && q(2) >0 && q(2) < size(map,1)
    if map(q(2,1),q(1,1)) == 1
        colli = false;
    end
end
end

function [q,index,Tree] = nearest(qin,Tree,dim)
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
    index = Tree(3,m(1));
    Tree(:,k) = [];
end
