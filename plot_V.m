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
close all
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
hold on;plot(y,V_value,'k','linewidth',2);
ylabel('Barrier function V','fontsize',14)
xlabel('y','fontsize',14)

theta_value=linspace(-pi,pi,20);
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
    if i==1
        style=':k';
    elseif i==2
        style='+k';
    else
        style='*k';
    end
o(i)=plot(theta_value,V(i,:),style,'linewidth',2)
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

alphaVal = 0.3;
h=surf(x,y,V_valeur.','edgecolor','none','FaceAlpha',alphaVal);

hold on
h1=plot3(50,100,V_value(1),'.k','markersize',20);
h2=plot3(Obstacle(:,1),Obstacle(:,2),V_obst,'.k','markersize',25);
h3=plot3(Destination(1),Destination(2),V_dest,'*k','markersize',15);
xlabel('x','fontsize',14)
ylabel('y','fontsize',14)
zlabel('V(s)','fontsize',14)
c_map=colormap([0.1 0.1 0.1;0.5 0.5 0.5])
%colormap winter;
caxis([-5 5]*1e-5)
%legend('>0','<0')
leg=legend([h1 h2 h3],{'Start','Obstacles','Destination'},'location','best');
view(gca,[-81.5 38]);
leg.FontSize=14;
% cb = colorbar(gca); %// This time we need a handle to the colorbar
% cb.Visible='off';
% annotation('textbox',...
%      [cb.Position(1:2) [cb.Position(3) 0.5*cb.Position(4)]],...
%      'FitBoxToText','off',...
%      'EdgeColor',[1 1 1],...
%      'BackgroundColor',(1-alphaVal)*[1 1 1]+alphaVal*[0.1 0.1 0.1]);

% cb.Ticks = (hSp.CLim(1):hSp.CLim(2))+0.5; %// Set the tick positions
% cb.TickLabels = fakeNames; %// Set the tick strings


a = get(gca, 'ZLim');
        
% Always put contour below the plot.
zpos = a(1);
possible_puissance=[1e-10 1e-9 1e-8 1e-7 1e-6 1e-5 1e-4 1e-3 1e-2 1e-1 1 1e1 1e2 1e3];
puissance_ok=0;
i=length(possible_puissance);
Max_valueee=max(max(V_valeur))
while puissance_ok~=1
if possible_puissance(i)>Max_valueee
    i=i-1;
else
    puissance=possible_puissance(i-1);
    puissance_ok=1;
end
end
puissance
%puissance=1e-6;
min(min(V_valeur));
Borne_bas=round(min(min(1/puissance*V_valeur)))
max(max(V_valeur));
Borne_haut=round(max(max(1/puissance*V_valeur)))
number_curve=10;
dcurve1=abs(round((-2*Borne_bas)/number_curve))
dcurve2=abs(round((2*Borne_haut)/number_curve))
level=[(Borne_bas:dcurve1:0-puissance) 0 (dcurve1:dcurve1:Borne_haut)]
% Get D contour data
[C, hh] = contour3(x,y,V_valeur.',number_curve);
hh.LineColor='k';
hh.LevelList=puissance*level;
%clabel(C,hh)
% % size zpos to match the data
% for i = 1 : length(hh)
%     zz = get(hh(i), 'ZData');
%     set(hh(i), 'ZData', zpos * ones(size(zz)));
% end

% %[C,h]=contour(x,y,V_valeur.',8,'LineColor','k','LevelList',1e-6*(-15:1:15));


end