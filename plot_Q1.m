clc;clear all;close all;
%% 1. obs_behind

%% 1.1 u=0
u=0
file='obs_behind';
C=[120.65713899705
-0.00020980787288420584
-1.330998136651402
0.033930893684380146
5.728696927403165e-6
-1.918206817126537e-6
-2.4424647381544665e-5
-3.145430617167242e-7
7.0591757446597305e-6
1.959303380905814];

%start: (50,100,0) [x,y,theta]
%destination: (50,190) [x,y]
%obstacle:(40,90)(60,90)

x=50;
y=linspace(100,190,300);
for i=1:length(y)
    Vec=[1 x y(i) 0 x*y(i) 0 0 x^2 y(i)^2 0];
    V_value(i)=Vec*C;
end
figure('color','white','name',['Evolution of V S_safe selon y u=' num2str(u) ' ' file])
hold on;plot(y,V_value,'linewidth',2);
ylabel('Barrier function V','fontsize',14)
xlabel('y','fontsize',14)

theta_value=linspace(-pi,pi,200);
Obstacle=[40 90;60 90];
for l=1:length(theta_value)
    theta=theta_value(l);
    for i=1:size(Obstacle,1)
        x_o=Obstacle(i,1);
        y_o=Obstacle(i,2);
        Vec=[1 x_o y_o theta x_o*y_o x_o*theta y_o*theta x_o^2 y_o^2 theta^2];
        V(i,l)=Vec*C;
    end
end
figure('color','white','name',['Evolution of V S_unsafe selon y u=' num2str(u) ' ' file])
hold on
for i=1:size(Obstacle,1)
plot(theta_value,V(i,:),'linewidth',2)
end
ylabel('Barrier function V','fontsize',14)
xlabel('\theta','fontsize',16)
l=legend('Obstacle1','Obstacle2','location','best');
l.FontSize=14;




%% 1.2 u=pi
clear all;
u=pi;
file='obs_behind';
C=[9.028479637373518
0.098731976072216
-0.1404025399844011
-0.23371024336914428
-7.882313770210235e-7
-2.5119080617453444e-9
1.8149638031177633e-6
3.9788739981840716e-7
8.811235976556649e-7
0.04533746908523554];

%start: (50,100,0) [x,y,theta]
%destination: (50,190) [x,y]
%obstacle:(40,90)(60,90)

x=50;
y=linspace(100,190,300);
for i=1:length(y)
    Vec=[1 x y(i) 0 x*y(i) 0 0 x^2 y(i)^2 0];
    V_value(i)=Vec*C;
end
figure('color','white','name',['Evolution of V S_safe selon y u=' num2str(u) ' ' file])
hold on;plot(y,V_value,'linewidth',2);
ylabel('Barrier function V','fontsize',14)
xlabel('y','fontsize',14)

theta_value=linspace(-pi,3*pi,200);
Obstacle=[40 90;60 90];
for l=1:length(theta_value)
    theta=theta_value(l);
    for i=1:size(Obstacle,1)
        x_o=Obstacle(i,1);
        y_o=Obstacle(i,2);
        Vec=[1 x_o y_o theta x_o*y_o x_o*theta y_o*theta x_o^2 y_o^2 theta^2];
        V(i,l)=Vec*C;
    end
end
figure('color','white','name',['Evolution of V S_unsafe selon y u=' num2str(u) ' ' file])
hold on
for i=1:size(Obstacle,1)
plot(theta_value,V(i,:),'linewidth',2)
end
ylabel('Barrier function V','fontsize',14)
xlabel('\theta','fontsize',16)
l=legend('Obstacle1','Obstacle2','location','best');
l.FontSize=14;

%% 2. obs_behind_side
%start: (50,100,0) [x,y,theta]
%destination: (50,190) [x,y]
%obstacle:(40,90)(60,90)
%new obstacle: (60,110)

%% 2.1 u=0
clear all;
u=0;
file='obs_behind_side';
C=[5.117315009194212
0.07755660613096096
-0.09523569394983065
0.040269770796462316
1.1927979295599172e-5
-0.0006815854596605201
-5.376121577536485e-5
0.00013174236056992205
1.3449687283351085e-5
1.7936557140762337];

%start: (50,100,0) [x,y,theta]
%destination: (50,190) [x,y]
%obstacle:(40,90)(60,90)(60,110)

x=50;
y=linspace(100,190,300);
for i=1:length(y)
    Vec=[1 x y(i) 0 x*y(i) 0 0 x^2 y(i)^2 0];
    V_value(i)=Vec*C;
end
figure('color','white','name',['Evolution of V S_safe selon y u=' num2str(u) ' ' file])
hold on;plot(y,V_value,'linewidth',2);
ylabel('Barrier function V','fontsize',14)
xlabel('y','fontsize',14)

theta_value=linspace(-pi,pi,200);
Obstacle=[40 90;60 90;60 110];
for l=1:length(theta_value)
    theta=theta_value(l);
    for i=1:size(Obstacle,1)
        x_o=Obstacle(i,1);
        y_o=Obstacle(i,2);
        Vec=[1 x_o y_o theta x_o*y_o x_o*theta y_o*theta x_o^2 y_o^2 theta^2];
        V(i,l)=Vec*C;
    end
end
figure('color','white','name',['Evolution of V S_unsafe selon y u=' num2str(u) ' ' file])
hold on
for i=1:size(Obstacle,1)
plot(theta_value,V(i,:),'linewidth',2)
end
ylabel('Barrier function V','fontsize',14)
xlabel('\theta','fontsize',16)
l=legend('Obstacle1','Obstacle2','Obstacle3','location','best');
l.FontSize=14;

%% 2.2 u=pi
clear all;
u=pi;
file='obs_behind_side';
C=[13.247100252455574
0.2586116934607574
-0.2642376141700866
-0.015292038525785825
6.392229431840312e-6
8.88841923662533e-6
2.549899217321617e-5
8.381804946381487e-6
1.9216046098947488e-5
0.09872912465444833];

%start: (50,100,0) [x,y,theta]
%destination: (50,190) [x,y]
%obstacle:(40,90)(60,90)(60,110)

x=50;
y=linspace(100,190,300);
for i=1:length(y)
    Vec=[1 x y(i) 0 x*y(i) 0 0 x^2 y(i)^2 0];
    V_value(i)=Vec*C;
end
figure('color','white','name',['Evolution of V S_safe selon y u=' num2str(u) ' ' file])
hold on;plot(y,V_value,'linewidth',2);
ylabel('Barrier function V','fontsize',14)
xlabel('y','fontsize',14)

theta_value=linspace(-pi,pi,200);
Obstacle=[40 90;60 90;60 110];
for l=1:length(theta_value)
    theta=theta_value(l);
    for i=1:size(Obstacle,1)
        x_o=Obstacle(i,1);
        y_o=Obstacle(i,2);
        Vec=[1 x_o y_o theta x_o*y_o x_o*theta y_o*theta x_o^2 y_o^2 theta^2];
        V(i,l)=Vec*C;
    end
end
figure('color','white','name',['Evolution of V S_unsafe selon y u=' num2str(u) ' ' file])
hold on
for i=1:size(Obstacle,1)
plot(theta_value,V(i,:),'linewidth',2)
end
ylabel('Barrier function V','fontsize',14)
xlabel('\theta','fontsize',16)
l=legend('Obstacle1','Obstacle2','Obstacle3','location','best');
l.FontSize=14;


%% Reste à montrer en 2D contour je pense...