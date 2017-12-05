include("optimisation1.jl")
fichier="obs_behind"
Data1=loadDataFromFile(fichier)
small_epsilon=0.000001
#start: (50,100,0) [x,y,theta]
#destination: (50,190) [x,y]
#obstacle: x_obstacle1=Data1.obstacles[1][1]
#obstacle: y_obstacle1=Data1.obstacles[1][2]
#obstacles are behind: (40,90) et (60,90) [90<100]

#le nombre d'obstacle
Nbr_obstacle=size(Data1.obstacles,1)

u=0 #test for pi after
v=6
K=0.2
b=0
w=0
Time_of_fligth=20
Time_step=0.1
Number_t_step=100
Time_of_fligth=Time_step*Number_t_step
#L=obstacle_near(fichier,[70 90])
k=100
u=-pi:(2*pi/(k-1)):pi
Current_time=0
u_possible=zeros(k)
s_suivant=zeros(k,3)
s_final=[Data1.destination[1] Data1.destination[2]]
s_connu=[Data1.start[1] Data1.start[2] Data1.start[3]]
s_enreg=zeros(Number_t_step,3)
for t=1:1:Number_t_step
	u_possible=0
	Current_time+=Time_step
	s_actual=s_connu
	mes_obstacles=obstacle_near(fichier,s_actual)

	for i_u=1:1:k
		i_u=round(i_u)
		s_dot=[-v*sin(s_actual[3])+w v*cos(s_actual[3]) -K*(s_actual[3]-u[i_u])]

		s_suivant[i_u,:]=s_actual+Time_step*s_dot
		C=SDP_barrier(mes_obstacles,s_actual,u[i_u])
		println(C)
		#check if s_suivant is in the safe zone
		x=s_suivant[i_u,1]
		y=s_suivant[i_u,2]
		theta=s_suivant[i_u,3]
		Vec=[1 x y theta x*y x*theta y*theta x^2 y^2 theta^2]
		println("Vec:",Vec,"\n\n")
		println(C*Vec.',"\n \n")
		prod_scal=C*Vec.'
		if (prod_scal[1]<0)
			u_possible=[u_possible i_u] #indice des u possibles
		end
	end
	println(u_possible)
	s_possible=s_suivant[u_possible[2:length(u_possible)],:]
	alpha=atan((-s_actual[2]+s_final[2])/(s_final[1]-s_actual[1]))
	if (-s_actual[1]+s_final[1])/norm(s_final[1:2]-s_actual[1:2])>=0 #cos>0
		alpha=alpha
	else
		if (-s_actual[2]+s_final[2])/norm(s_final[1:2]-s_actual[1:2])>=0 #sin>0
			alpha=pi+alpha
		else
			alpha=alpha-pi
		end
	end
	alpha=alpha-pi/2
	println(s_possible[:,3])
	Dot_prod=(s_possible[:,3]-alpha)
	println(Dot_prod)
	optimum=extrema(abs.(Dot_prod))
	min=optimum[1]
	println(min)
	Indice_u=find_index(Dot_prod,min)
	println(Indice_u)
	println(u_possible)
	println(u_possible[Indice_u+1])
	u_suivant=u[u_possible[Indice_u+1]]
	s_dot=[-v*sin(s_actual[3])+w v*cos(s_actual[3]) -K*(s_actual[3]-u_suivant)]
	println(s_actual)
	println(s_connu)
	s_connu=s_actual+Time_step*s_dot
	s_enreg[t,:]=s_connu
	u_possible=[0]
end
