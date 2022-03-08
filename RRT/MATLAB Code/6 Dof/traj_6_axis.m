clc; clear; close all;

%% 六轴机械臂
robot = loadrobot("abbIrb120T","DataFormat","column","Gravity",[0 0 -9.81]);
tcp = removeBody(robot, 'tool0');
sting = rigidBody('sting');
jnt_sting = rigidBodyJoint('jnt_sting', 'fixed');
sting1 = rigidBody('sting1');
jnt_sting1 = rigidBodyJoint('jnt_sting1', 'fixed');
setFixedTransform(jnt_sting,[0.1, 0, 0, 0],'dh');
setFixedTransform(jnt_sting1,[0.05, 0, 0, 0],'dh');
sting.Joint = jnt_sting;
sting1.Joint = jnt_sting1;
addBody(robot,sting,'link_6');
addBody(robot,sting1,'link_6');
addSubtree(robot,'sting',tcp);
collisionObj = collisionCylinder(0.005,0.099);
collisionObj.Pose = axang2tform([0 1 0 pi/2]);
addCollision(robot.Bodies{9},collisionObj)
% 前三轴顺序为 z,x,y
% 查看细节 部件名,关节名 .索引序号
showdetails(robot)

% 获取机器人的home构型
conf= homeConfiguration(robot);

% show可视化结果在figure中显示,此结果是默认配置的机器人状态	
% show(robot,conf);

numJoints = numel(homeConfiguration(robot));
%iviz = interactiveRigidBodyTree(robot,"MarkerBodyName","tool0");
homefig = homeConfiguration(robot);
home = [0; 0; 0; 0; 0; 0];
%NarrConfig = [0.5; 0; 0; 0; -1*pi/8; -pi/4; -0.9; -0.16; -3];
show(robot,home,"Visuals","on","Collisions","on");   
hold on
positionConst = constraintPositionTarget("tool0");

%% 障碍物设置
ax = gca;

plane = collisionBox(2,2,0.2);
plane.Pose = trvec2tform([0.5 0 -0.1]);

% floor
floor = collisionBox(1.5, 1.5, 0.2);
floor.Pose = trvec2tform([0.5 0 -0.1]);
[~, patchObj] = show(floor,'Parent',ax);
patchObj.FaceColor = [1 0.5 0.1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%工件1
%down
down1 = collisionBox(0.5, 0.5, 0.01);
down1.Pose = trvec2tform([0.5 0 0.005]);
[~, patchObj] = show(down1,'Parent',ax);
patchObj.FaceColor = [1 1 1];
%right
left1 = collisionBox(0.2, 0.01, 0.1);
left1.Pose = trvec2tform([0.35 -0.245 0.06]);
[~, patchObj] = show(left1,'Parent',ax);
patchObj.FaceColor = [0 1 1];
%right
right1 = collisionBox(0.2, 0.01, 0.1);
right1.Pose = trvec2tform([0.35 0.245 0.06]);
[~, patchObj] = show(right1,'Parent',ax);
patchObj.FaceColor = [1 0 1];

env = {plane,floor,down1,left1,right1};

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%——————————————————————————————————————————————————%%%%%
%%%%——————————————————————开始计算————————————————————————%%%%%
%%%%——————————————————————————————————————————————————%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 
pt1 = [0.25, 0.235, 0.01];
pt2 = [0.45, 0.235, 0.01];
safetypt1 = (pt1+pt2)/2 + [0, -0.05, 0.15];
pt3 = [0.25, -0.235, 0.01];
pt4 = [0.45, -0.235, 0.01];
safetypt2 = (pt3+pt4)/2 + [0, 0.05, 0.15];
start_pt = pt1;
goal_pt = pt3;

%% 逆运动学计算起始关节坐标
gik1 = generalizedInverseKinematics;
gik1.RigidBodyTree = robot;
gik1.ConstraintInputs = {'position','aiming'};
posTgt = constraintPositionTarget('tool0');
posTgt.TargetPosition = safetypt1;

aimCon = constraintAiming('tool0');
aimCon.TargetPoint = [-0.5, 0.5, -0.5];

q0 = homeConfiguration(robot); % Initial guess for solver
[start_config,solutionInfo] = gik1(q0,posTgt,aimCon);

show(robot,start_config);
title(['Solver status: ' solutionInfo.Status])
axis([-0.75 0.75 -0.75 0.75 -0.5 1])


%% 逆运动学计算目标关节坐标
aik = analyticalInverseKinematics(robot);
showdetails(aik)

aik.KinematicGroup
generateIKFunction(aik,'robotIK');
eePosition = safetypt2;
eePose = trvec2tform(eePosition);
hold on
plotTransforms(eePosition,tform2quat(eePose))
hold off
ikConfig = robotIK(eePose); % Uses the generated file

show(robot,ikConfig(1,:));
hold on
plotTransforms(eePosition,tform2quat(eePose))
hold off

gripperPosition_start = tform2trvec(getTransform(robot,start_config,robot.Bodies{1,10}.Name));
plot3(gripperPosition_start(1),gripperPosition_start(2),gripperPosition_start(3),'*','Color','red');
hold on
gripperPosition_goal = tform2trvec(getTransform(robot,goal_config,robot.Bodies{1,10}.Name));
plot3(gripperPosition_goal(1),gripperPosition_goal(2),gripperPosition_goal(3),'*','Color','red');
hold on


%%

planner = manipulatorRRT(robot, env);

planner.MaxConnectionDistance = 0.3;
planner.ValidationDistance = 0.1;

startConfig = start_config';
goalConfig = goal_config';

rng('default');
path = plan(planner,startConfig,goalConfig);

interpStates = interpolate(planner, path);

for i = 1:2:size(interpStates,1)
    show(franka, interpStates(i,:),...
        "PreservePlot", false,...
        "Visuals","off",...
        "Collisions","on");
    title("Plan 1: MaxConnectionDistance = 0.3")
    drawnow;
end