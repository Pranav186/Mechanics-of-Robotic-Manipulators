%%
clc
clear all

% Part 1 CCC path
ss = stateSpaceDubins([-50 -50; 50 50; -pi pi]);
startPose = [0 0 0];
waypoints=[2 3 pi/2;
           5 8 pi/4;
           8 10 pi/8;
           10 13 -pi/2;
           20 15 -pi/6;
           22 18 -pi/4;
           26 22 0;
           27 28 pi/4];
goalPose = [0 0 0];

ss.MinTurningRadius = 0.1*max(max(waypoints(:,1))-min((waypoints(:,1)),max(waypoints(:,2))-min((waypoints(:,2)))));

%create path object
pathobj = navPath(ss);
append(pathobj, waypoints); %%append(path,states)
interpolate(pathobj,200);%%interpolate(path,numState>number of exsisting way point)

%create dubins object
dubConnObj = dubinsConnection('DisabledPathTypes',["LSL","RSR","RSL","LSR"]);%for CCC
dubConnObj.MinTurningRadius = 0.1*max(max(waypoints(:,1))-min((waypoints(:,1)),max(waypoints(:,2))-min((waypoints(:,2)))));

pathCosts=zeros(9,1);
figure
grid on;
axis equal;
[pathSegObj, pathCosts(1)] = connect(dubConnObj,startPose,waypoints(1,:));
show(pathSegObj{1});
hold on
for i=1:1:7
   [pathSegObj, pathCosts(i+1)] = connect(dubConnObj,waypoints(i,:),waypoints(i+1,:));
    show(pathSegObj{1});
    hold on;
    plot(waypoints(i,1),waypoints(i,2),"*r", "Markersize", 10);
    hold on 
end
plot(waypoints(8,1),waypoints(8,2),"*r", "Markersize", 10);
hold on
[pathSegObj, pathCosts(9)] = connect(dubConnObj,waypoints(8,:),goalPose);
show(pathSegObj{1});
plot(goalPose(1,1),goalPose(1,2),"*r", "Markersize", 10);
hold on 
pathCosts
sum(pathCosts)
