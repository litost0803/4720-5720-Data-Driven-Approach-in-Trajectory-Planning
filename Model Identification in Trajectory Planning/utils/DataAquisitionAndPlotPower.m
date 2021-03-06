
%% sindy
%%without obstacle avoidance
a1 = [    -0.0548
    0.4476
   -1.4612
    0.1473
   -0.7461
    1.0603
    0.0614
   -0.1800
    0.0491
    ];

%%with obstacle avoidance
a2 = [   
    0.0320
   -0.0295
   -0.8545
    0.0182
   -0.2300
    0.5442
   -0.0609
    0.3089
   -0.4398];



%%analytical
%%without obstacle avoidance
a3 = [      0.0457
   -0.0321
   -0.9040
    0.1472
   -0.7460
    1.0602
    0.0614
   -0.1800
    0.0491
    ];

%%with obstacle avoidance
a4 = [     0.1155
   -0.3810
   -0.4852
    0.1473
   -0.7461
    1.0603
    0.0614
   -0.1800
    0.0491
];



theta_0 = [pi/2, pi/5, -pi/4];      % initial position
d_theta_0 = [0, 0, 0];
theta_end = [-pi/2, 2*pi/5, -pi/3]; % final position



[P1, E1,T,TrainingData] = DataGeneration(a1,theta_0,d_theta_0);
[P2, E2] = DataGeneration(a2,theta_0,d_theta_0);
[P3, E3] = DataGeneration(a3,theta_0,d_theta_0);
[P4, E4] = DataGeneration(a4,theta_0,d_theta_0);

hold on;
grid on;
plot(T,P1);
hold on;
plot(T,P2);
hold on;
plot(T,P3);
hold on;
plot(T,P4);
legend(['MPEO:' num2str(E1)],['MPOA:' num2str(E2)],['AEO:' num2str(E3)],['AOA:' num2str(E4)] )

xlabel('t(s)');
ylabel('Total Consumed Power(W)');

hold on


%%% training data can be obtained from several joint space trajectories,
%%% the varaible TrainingData is for data acquisition
%%% for training, using data from random trajectories
%%% total power data can also be be obained from this function.

function [P, E,T,TrainingData] = DataGeneration(a,theta_0,d_theta_0)

P = [];
P_1 = [];
P_2 = [];
P_3 = [];

T = [];
E = 0;
VELO = [];
ACCE = [];
ANGLE = [];
Torque = [];
for I_t = 0:0.005:2
    
[p, p_sp] = energy(I_t,a,theta_0, d_theta_0);
E = E + p * 0.005;

P = [P; p]; %%% total power
P_1 = [P_1; p_sp(1)]; %%% for each joint
P_2 = [P_2; p_sp(2)];
P_3 = [P_3; p_sp(3)];

ANG= joint_space(I_t, a, theta_0, d_theta_0)';
ANGLE = [ANGLE; ANG];

Torq = geForce(I_t, a, theta_0, d_theta_0)';
Torque = [Torque; [Torq]];

V_T = geVelo(I_t, a, theta_0, d_theta_0)';
VELO = [VELO; [V_T]];

AC = geAcce(I_t, a)';
ACCE = [ACCE; [AC]];
T = [T; I_t];

end

TrainingData = [Torque'; ANGLE'; VELO'; ACCE']'; %%%training data


end




function [e,e_sp] = energy(I_t,a,theta_0, d_theta_0)

M = geForce(I_t,a,theta_0, d_theta_0);
v = geVelo(I_t, a, theta_0, d_theta_0);

e_sp = M.*v;
e = sum(abs(e_sp));

end


function [theta] = joint_space(t, a, theta_0, d_theta_0)

a5 = theta_0(1);
a10 = theta_0(2);
a15 = theta_0(3);


a4 = d_theta_0(1);
a9 = d_theta_0(2);
a14 = d_theta_0(3);


theta = zeros(3, 1);
theta(1) = a(1)*t^4 + a(2)*t^3 + a(3)*t^2 + a4*t +a5;
theta(2) = a(4)*t^4 + a(5)*t^3 + a(6)*t^2 + a9*t +a10;
theta(3) = a(7)*t^4 + a(8)*t^3 + a(9)*t^2 + a14*t +a15;


end

function [M] = geForce(t, a, theta_0, d_theta_0)

a5 = theta_0(1);
a10 = theta_0(2);
a15 = theta_0(3);


a4 = d_theta_0(1);
a9 = d_theta_0(2);
a14 = d_theta_0(3);


a1 = a(1);
a2 = a(2);
a3 = a(3);
a6 = a(4);
a7 = a(5);
a8 = a(6) ;
a11 = a(7);
a12 = a(8);
a13 = a(9) ;


M = zeros(3,1);

M(1) =  (76779614644346195*a3)/4611686018427387904 + (3934632864847017*cos(2*a10 + a15 + 2*a9*t + a14*t + 2*a6*t^4 + 2*a7*t^3 + 2*a8*t^2 + a11*t^4 + a12*t^3 + a13*t^2)*(12*a1*t^2 + 6*a2*t + 2*a3))/576460752303423488 + (230338843933038585*a2*t)/4611686018427387904 + (3934632864847017*cos(a11*t^4 + a12*t^3 + a13*t^2 + a14*t + a15)*(12*a1*t^2 + 6*a2*t + 2*a3))/576460752303423488 + (1259891002956151*cos(2*a10 + 2*a15 + 2*a9*t + 2*a14*t + 2*a6*t^4 + 2*a7*t^3 + 2*a8*t^2 + 2*a11*t^4 + 2*a12*t^3 + 2*a13*t^2)*(12*a1*t^2 + 6*a2*t + 2*a3))/576460752303423488 + (230338843933038585*a1*t^2)/2305843009213693952 + (27033242271419503*cos(2*a6*t^4 + 2*a7*t^3 + 2*a8*t^2 + 2*a9*t + 2*a10)*(12*a1*t^2 + 6*a2*t + 2*a3))/4611686018427387904 - (27033242271419503*sin(2*a6*t^4 + 2*a7*t^3 + 2*a8*t^2 + 2*a9*t + 2*a10)*(4*a1*t^3 + 3*a2*t^2 + 2*a3*t + a4)*(4*a6*t^3 + 3*a7*t^2 + 2*a8*t + a9))/2305843009213693952 - (3934632864847017*sin(2*a10 + a15 + 2*a9*t + a14*t + 2*a6*t^4 + 2*a7*t^3 + 2*a8*t^2 + a11*t^4 + a12*t^3 + a13*t^2)*(4*a1*t^3 + 3*a2*t^2 + 2*a3*t + a4)*(4*a6*t^3 + 3*a7*t^2 + 2*a8*t + a9))/288230376151711744 - (3934632864847017*sin(2*a10 + a15 + 2*a9*t + a14*t + 2*a6*t^4 + 2*a7*t^3 + 2*a8*t^2 + a11*t^4 + a12*t^3 + a13*t^2)*(4*a1*t^3 + 3*a2*t^2 + 2*a3*t + a4)*(4*a11*t^3 + 3*a12*t^2 + 2*a13*t + a14))/576460752303423488 - (3934632864847017*sin(a11*t^4 + a12*t^3 + a13*t^2 + a14*t + a15)*(4*a1*t^3 + 3*a2*t^2 + 2*a3*t + a4)*(4*a11*t^3 + 3*a12*t^2 + 2*a13*t + a14))/576460752303423488 - (1259891002956151*sin(2*a10 + 2*a15 + 2*a9*t + 2*a14*t + 2*a6*t^4 + 2*a7*t^3 + 2*a8*t^2 + 2*a11*t^4 + 2*a12*t^3 + 2*a13*t^2)*(4*a1*t^3 + 3*a2*t^2 + 2*a3*t + a4)*(4*a6*t^3 + 3*a7*t^2 + 2*a8*t + a9))/288230376151711744 - (1259891002956151*sin(2*a10 + 2*a15 + 2*a9*t + 2*a14*t + 2*a6*t^4 + 2*a7*t^3 + 2*a8*t^2 + 2*a11*t^4 + 2*a12*t^3 + 2*a13*t^2)*(4*a1*t^3 + 3*a2*t^2 + 2*a3*t + a4)*(4*a11*t^3 + 3*a12*t^2 + 2*a13*t + a14))/288230376151711744;

M(2) = (37308366950851875*a8)/1152921504606846976 + (1259891002956151*a13)/144115188075855872 + (48377442017232500631*cos(a6*t^4 + a7*t^3 + a8*t^2 + a9*t + a10))/56294995342131200000 + (4130859204211177*cos(a10 + a15 + a9*t + a14*t + a6*t^4 + a7*t^3 + a8*t^2 + a11*t^4 + a12*t^3 + a13*t^2))/9007199254740992 + (1259891002956151*sin(2*a10 + 2*a15 + 2*a9*t + 2*a14*t + 2*a6*t^4 + 2*a7*t^3 + 2*a8*t^2 + 2*a11*t^4 + 2*a12*t^3 + 2*a13*t^2)*(4*a1*t^3 + 3*a2*t^2 + 2*a3*t + a4)^2)/576460752303423488 + (111925100852555625*a7*t)/1152921504606846976 + (3779673008868453*a12*t)/144115188075855872 + (3934632864847017*cos(a11*t^4 + a12*t^3 + a13*t^2 + a14*t + a15)*(12*a6*t^2 + 6*a7*t + 2*a8))/288230376151711744 + (3934632864847017*cos(a11*t^4 + a12*t^3 + a13*t^2 + a14*t + a15)*(12*a11*t^2 + 6*a12*t + 2*a13))/576460752303423488 + (27033242271419503*sin(2*a6*t^4 + 2*a7*t^3 + 2*a8*t^2 + 2*a9*t + 2*a10)*(4*a1*t^3 + 3*a2*t^2 + 2*a3*t + a4)^2)/4611686018427387904 + (111925100852555625*a6*t^2)/576460752303423488 + (3779673008868453*a11*t^2)/72057594037927936 + (3934632864847017*sin(2*a10 + a15 + 2*a9*t + a14*t + 2*a6*t^4 + 2*a7*t^3 + 2*a8*t^2 + a11*t^4 + a12*t^3 + a13*t^2)*(4*a1*t^3 + 3*a2*t^2 + 2*a3*t + a4)^2)/576460752303423488 - (3934632864847017*sin(a11*t^4 + a12*t^3 + a13*t^2 + a14*t + a15)*(4*a11*t^3 + 3*a12*t^2 + 2*a13*t + a14)^2)/576460752303423488 - (3934632864847017*sin(a11*t^4 + a12*t^3 + a13*t^2 + a14*t + a15)*(4*a6*t^3 + 3*a7*t^2 + 2*a8*t + a9)*(4*a11*t^3 + 3*a12*t^2 + 2*a13*t + a14))/288230376151711744;
 
M(3) =  (1259891002956151*a8)/144115188075855872 + (328981078484943515*a13)/36893488147419103232 + (4130859204211177*cos(a10 + a15 + a9*t + a14*t + a6*t^4 + a7*t^3 + a8*t^2 + a11*t^4 + a12*t^3 + a13*t^2))/9007199254740992 + (1259891002956151*sin(2*a10 + 2*a15 + 2*a9*t + 2*a14*t + 2*a6*t^4 + 2*a7*t^3 + 2*a8*t^2 + 2*a11*t^4 + 2*a12*t^3 + 2*a13*t^2)*(4*a1*t^3 + 3*a2*t^2 + 2*a3*t + a4)^2)/576460752303423488 + (3779673008868453*a7*t)/144115188075855872 + (986943235454830545*a12*t)/36893488147419103232 + (3934632864847017*cos(a11*t^4 + a12*t^3 + a13*t^2 + a14*t + a15)*(12*a6*t^2 + 6*a7*t + 2*a8))/576460752303423488 + (3779673008868453*a6*t^2)/72057594037927936 + (986943235454830545*a11*t^2)/18446744073709551616 + (3934632864847017*sin(2*a10 + a15 + 2*a9*t + a14*t + 2*a6*t^4 + 2*a7*t^3 + 2*a8*t^2 + a11*t^4 + a12*t^3 + a13*t^2)*(4*a1*t^3 + 3*a2*t^2 + 2*a3*t + a4)^2)/1152921504606846976 + (3934632864847017*sin(a11*t^4 + a12*t^3 + a13*t^2 + a14*t + a15)*(4*a1*t^3 + 3*a2*t^2 + 2*a3*t + a4)^2)/1152921504606846976 + (3934632864847017*sin(a11*t^4 + a12*t^3 + a13*t^2 + a14*t + a15)*(4*a6*t^3 + 3*a7*t^2 + 2*a8*t + a9)^2)/576460752303423488;

end

function [v] = geVelo(t, a, theta_0, d_theta_0)

a4 = d_theta_0(1);
a9 = d_theta_0(2);
a14 = d_theta_0(3);


a1 = a(1);
a2 = a(2);
a3 = a(3);
a6 = a(4);
a7 = a(5);
a8 = a(6) ;
a11 = a(7);
a12 = a(8);
a13 = a(9) ;


v = zeros(3,1);
v(1) = 4*a1*t^3 + 3*a2*t^2 + 2*a3*t + a4;
v(2) = 4*a6*t^3 + 3*a7*t^2 + 2*a8*t + a9;
v(3) = 4*a11*t^3 + 3*a12*t^2 + 2*a13*t + a14;

end

function [A] = geAcce(t, a)


a1 = a(1);
a2 = a(2);
a3 = a(3);
a6 = a(4);
a7 = a(5);
a8 = a(6) ;
a11 = a(7);
a12 = a(8);
a13 = a(9) ;


A = zeros(3,1);
A(1) = 12*a1*t^2 + 6*a2*t + 2*a3;
A(2) = 12*a6*t^2 + 6*a7*t + 2*a8;
A(3) = 12*a11*t^2 + 6*a12*t + 2*a13;




end
