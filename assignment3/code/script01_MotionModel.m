% demo08_squareWorldMCL
% Demo generic Monte Carlo Localization

path(path,'threedee') 

%% Create a map
A = [
    0,0,2.5,4
    1.5,4,1,2
    1.5,6,1,2
    .25,8,2.25,3
    2.5,11,2,.75
    4.5,11,2,.75
    6.25,4,1.5,4
    2.4,-.25,10,.25
    4,-2,1,1.75
    ]
A = A./2
map=SquareMap(A)

%% and a robot with noisy odometry
V=diag([0.01, 0.1*pi/180].^2)
veh=Differential(V)
veh.add_driver(DeterministicPath('log-1423106178.txt'));

%% and then a sensor with noisy readings
W=0.05^2;
sensor = RangeSensor(veh,map, W,'log-1423106178.txt')

%% define two covariances for random noise Q and L (hmmm!)
% For Q, use the uncertainly estimates from A2!
Q = diag([0.01,0.01,0.1*pi/180].^2);
L = diag(0.1); 

%% Finally, construct ParticleFilter
pf = GenericParticleFilter(veh, sensor, Q, L, 200);

%% and run for 1000 steps
pf.run(1000,'nouniform','nosense');
