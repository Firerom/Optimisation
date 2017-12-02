%
%
%   plot_V(C,u,file):
%
%       display V in safe and in unsafe
%       file is a string 'obs_behind_side' or 'obs_behind'
%            or a number        2                   1  
%
%


function plot_V(C,u,file)
%start: (50,100,0) [x,y,theta]
%destination: (50,190) [x,y]
%obstacle:(40,90)(60,90)
if isnumeric(file)
    if file==1
        file='obs_behind';
    elseif file==2
        file='obs_behind_side';
    else
        fprintf('File does not exist\n');
        fprintf('file = 1 = ''obs_behind'' \n');
        fprintf('file = 2 = ''obs_behind_side'' \n');
        return
    end
end

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
if strcmpi(file,'obs_behind')
    Obstacle=[40 90;60 90];
elseif strcmpi(file,'obs_behind_side')
    Obstacle=[40 90;60 90;60 110];
end
V_obst=zeros(1,size(Obstacle,1));
for l=1:length(theta_value)
    theta=theta_value(l);
    for i=1:size(Obstacle,1)
        x_o=Obstacle(i,1);
        y_o=Obstacle(i,2);
        Vec=[1 x_o y_o theta x_o*y_o x_o*theta y_o*theta x_o^2 y_o^2 theta^2];
        V(i,l)=Vec*C;
        if l==1
            Vec=[1 x_o y_o 0 x_o*y_o x_o*0 y_o*0 x_o^2 y_o^2 0^2];
            V_obst(i)=Vec*C;
        end

    end
end
figure('color','white','name',['Evolution of V S_unsafe selon y u=' num2str(u) ' ' file])
hold on
for i=1:size(Obstacle,1)
o(i)=plot(theta_value,V(i,:),'linewidth',2)
end
ylabel('Barrier function V','fontsize',14)
xlabel('\theta','fontsize',16)

for i=1:size(Obstacle,1)
str_legend{i}=['Obstacle ' int2str(i) ': (' int2str(Obstacle(i,1)) ';' int2str(Obstacle(i,2)) ')']
end
l=legend(o,str_legend,'location','best');
l.FontSize=12;



x=linspace(0,100,200);
y=linspace(0,200,200);
for i=1:length(x)
    for j=1:length(y)
        Vec=[1 x(i) y(j) 0 x(i)*y(j) 0 0 x(i)^2 y(j)^2 0];
        V_valeur(i,j)=Vec*C;
    end
end


Destination=[50 190];
x_o=Destination(1);
y_o=Destination(2);
Vec=[1 x_o y_o 0 x_o*y_o x_o*0 y_o*0 x_o^2 y_o^2 0^2];
V_dest=Vec*C;

figure('color','white','name',['Evolution of V selon x,y,0 u=' num2str(u) ' ' file])



h=surfc(x,y,V_valeur.','edgecolor','none','FaceAlpha',.3);
hold on
h1=plot3(50,100,V_value(1),'.g','markersize',20);
h2=plot3(Obstacle(:,1),Obstacle(:,2),V_obst,'.r','markersize',20);
h3=plot3(Destination(1),Destination(2),V_dest,'.b','markersize',20);
xlabel('x','fontsize',14)
ylabel('y','fontsize',14)
zlabel('V(s)','fontsize',14)
colormap([0 0 0.7;0.7 0 0])
caxis([-5 5])
leg=legend([h1 h2 h3],{'Start','Obstacles','Destination'},'location','best');
view(gca,[-81.5 38]);
leg.FontSize=14;

end