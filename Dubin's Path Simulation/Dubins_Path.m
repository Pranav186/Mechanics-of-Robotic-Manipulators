
clc
clear all

%define points to be visited.First/Last points are start/destination points
points=[0 0;
           2 3;
           5 16;
           2 4;
           10 16;
           18 13;
           25 12;
           15 24;
           20 20];
%points=flip(points);

%to find the sequence of points to be visited
userConfig = struct('xy',points);
resultStruct = tspof_ga(userConfig); %function to calculate the sequence 
sequence=resultStruct.optRoute

%create waypoints matrix with same size as that of points matrix and copy first and last points
waypoints=zeros(size(points,1),size(points,2));
waypoints(1,:)=points(1,:);
waypoints(size(points,1),:)=points(size(points,1),:);

%arrange points in the optimal sequence
for i=1:length(sequence)
    waypoints(i+1,:)=points(sequence(i),:);
end

%add zero column for heading angles 
waypoints=[waypoints zeros(size(waypoints,1),1)];

%create statespace and define minimum radius
ss = stateSpaceDubins([-50 -50; 50 50; 0 2*pi]);
ss.MinTurningRadius = 0.1*max(max(waypoints(:,1))-min((waypoints(:,1)),max(waypoints(:,2))-min((waypoints(:,2)))));

%create path object
pathobj = navPath(ss);
append(pathobj, waypoints); %append(path,states)
interpolate(pathobj,200);%interpolate(path,numState>number of exsisting way point)

%create dubins object
dubConnObj = dubinsConnection();%for CSC
dubConnObj.MinTurningRadius = 0.1*max(max(waypoints(:,1))-min((waypoints(:,1)),max(waypoints(:,2))-min((waypoints(:,2)))));

%discretize the heading angles
theta=0:pi/25:2*pi; 

%define costs
Total_cost=0;
pathCosts=zeros(length(theta),length(theta));

%calculating optimal heading angles for the first 2 nodes
for node1=1:length(theta)
    waypoints(1,3)=theta(node1);
    for node2=1:length(theta)
        waypoints(2,3)=theta(node2);
        [pathSegObj, pathCosts(node1,node2)] = connect(dubConnObj,waypoints(1,:),waypoints(2,:));
    end
end

%find minimum value of cost and update total cost
M=min(pathCosts(:));
Total_cost=Total_cost+M;

%find index of M
[row,col]=find(M==pathCosts);

%use indices of M to get the heading angles of first and second point
waypoints(1,3)=theta(row);
waypoints(2,3)=theta(col);

%calculating heading angles for the remaining nodes
costs=zeros(8,length(theta));
for node=3:10
    for i=1:length(theta)
        waypoints(node,3)=theta(i);
        [pathSegObj, costs(node,i)] = connect(dubConnObj,waypoints(node-1,:),waypoints(node,:));
    end
    M=min(costs(node,:));
    Total_cost=Total_cost+M;
    [row,col]=find(M==costs);
    waypoints(node,3)=theta(col);
end

%diplaying the total cost
Total_cost
waypoints

%plotting the optimal dubins path
figure
grid on;
axis equal;
for i=1:8
   [pathSegObj, pathCosts] = connect(dubConnObj,waypoints(i,:),waypoints(i+1,:));
    show(pathSegObj{1});
    hold on;
    plot(waypoints(i,1),waypoints(i,2),"*r", "Markersize", 10);
    hold on 
end
plot(waypoints(10,1),waypoints(10,2),"*r", "Markersize", 10);
hold on
