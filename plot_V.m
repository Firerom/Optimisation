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
end