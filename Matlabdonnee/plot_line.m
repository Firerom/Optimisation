%%
clear all
close all 
filename='Donne_Romain.txt';
trajectory_name='trajectory_Romain.txt';
%% read of all the data 
% format file of the set of data:
% first 3 number are the positon and orination of the drone: xs,ys,thetas
% second 2 number are the position of the objectif:xi ,yi
% third there is the number of obstacle
% then fro each obstacle, we have 2 number : xi,yi

data=dlmread(filename);
start=data(1,:);
dest=data(2,1:2);
Nbr_obstacle=data(3,1);
Obstacle=zeros(Nbr_obstacle,2);
for i=1:Nbr_obstacle
    Obstacle(i,:)=data(3+i,1:2);
end

%% read of the actual trajectory
% contain x, y , theta at each iteration
S_tot=dlmread(trajectory_name);
S=S_tot(find(S_tot(:,1)~=0),:); %S=S_tot(find(S_tot~=0),2,3);

%% actual plot

figure
hold on 
plot(start(1),start(2),'go')
plot(dest(1),dest(2),'ro')

for i=1:Nbr_obstacle
    plot(Obstacle(i,1),Obstacle(i,2),'k*')
end

plot(S(:,1),S(:,2),'*')