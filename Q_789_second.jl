include("optimisation1.jl")
fichier="romain"
Data1=loadDataFromFile(fichier)
#Data1=generateData(4)
#obstacles = Array{Tuple{Float64,Float64}}(4)
#obstacles[1] = (40, 90)
#obstacles[2] = (60, 90)
#obstacles[3] = (47, 110)
#obstacles[4] = (50, 110)
#Data1 = Optinum.Data((50.0, 100.0, 0.0), (50.0, 190.0), obstacles)
tolerance=0.05
small_epsilon=0.000001
#start: (50,100,0) [x,y,theta]
#destination: (50,190) [x,y]
#obstacle: x_obstacle1=Data1.obstacles[1][1]
#obstacle: y_obstacle1=Data1.obstacles[1][2]
#obstacles are behind: (40,90) et (60,90) [90<100]

#le nombre d'obstacle
Nbr_obstacle=size(Data1.obstacles,1)
clear_console()
u=0 #test for pi after
v=6
K=0.2
b=0
w=-0.25
Time_of_fligth=20
Time_step=0.1
Number_t_step=400
Time_of_fligth=Time_step*Number_t_step
#L=obstacle_near(fichier,[70 90])
k=10
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
	mes_obstacles_near=obstacle_near(fichier,s_actual)

	mes_obstacles=mes_obstacles_near[1]
	number_obst_near=mes_obstacles_near[2]

	if(number_obst_near!=0)# il y a des obstacles
	#phase avec le calcul de SDP pour V  car obstacle
		for i_u=1:1:k
			i_u=round(i_u)
			s_dot=[-v*sin(s_actual[3])+w v*cos(s_actual[3]) -K*(s_actual[3]-u[i_u])]

			s_suivant[i_u,:]=s_actual+Time_step*s_dot
			C=SDP_barrier(mes_obstacles,s_actual,u[i_u])
			x=s_suivant[i_u,1]
			y=s_suivant[i_u,2]
			theta=s_suivant[i_u,3]
			c1=C[2]
			c2=C[3]
			c3=C[4]
			c4=C[5]
			c5=C[6]
			c6=C[7]
			c7=C[8]
			c8=C[9]
			c9=C[10]
			#grad_V=[(c1+c4*s_actual[2]+c5*s_actual[3]+2*c7*s_actual[1]),(c2+c4*s_actual[1]+c6*s_actual[3]+2*c8*s_actual[2]),(c3+c5*s_actual[1]+c6*s_actual[2]+2*c9*s_actual[3])]
			## test pour romain
			grad_V=[(c1+c4*s_suivant[2]+c5*s_suivant[3]+2*c7*s_suivant[1]),(c2+c4*s_suivant[1]+c6*s_suivant[3]+2*c8*s_suivant[2]),(c3+c5*s_suivant[1]+c6*s_suivant[2]+2*c9*s_suivant[3])]
			Vec=[1 x y theta x*y x*theta y*theta x^2 y^2 theta^2]
			prod_scal=C*Vec.'
			write_console("prod_scal:\n",C*Vec.')
			##
			write_console(C,"")
			write_console("V_dot:\n",dot(grad_V,s_dot))
			#if(dot(grad_V,s_dot)<=tolerance )
			## test pour romain
			if(dot(grad_V,s_dot)<=tolerance )
				if(prod_scal[1]<tolerance)
				u_possible=[u_possible i_u]
				end
			end
		end

	else
		#on peut tout de suite calculer tout les s(notamenet theta)pour tout les u
		#
		for i_u=1:1:k
			i_u=round(i_u)
			s_dot=[-v*sin(s_actual[3])+w v*cos(s_actual[3]) -K*(s_actual[3]-u[i_u])]
			s_suivant[i_u,:]=s_actual+Time_step*s_dot
			s_possible=s_suivant# tout es possible, car aucune contrainte
			x=s_suivant[i_u,1]
			y=s_suivant[i_u,2]
			theta=s_suivant[i_u,3]
			u_possible=[u_possible i_u]# on prend tout les indices

		end

	end

	#calule de meilleur u->joris
	write_console("u possible: ",u_possible)
	s_possible=s_suivant[u_possible[2:length(u_possible)],:]
 	write_console("s_suivant possible: ","")
	for i=1:size(s_possible,1)
		write_console(s_possible[i,:],"")
	end
	write_console("\nS suivant(dynamique deja fixee): ",[s_possible[1,1] s_possible[1,2]])
	write_console("S Final: ",s_final,"\narctg a realiser\n")

	alpha=mon_arctg(s_suivant[1,1:2],s_final)
	alpha=alpha-pi/2
	write_console("alpha= ",alpha)
	Aminimiser=s_possible[:,3]-alpha
	write_console("A minimiser vers 0: ",Aminimiser)
	optimum=extrema(abs.(Aminimiser))
	min=optimum[1]
	Indice_u=find_index(abs.(Aminimiser),min)
	write_console("Choosen index: ",Indice_u)
	s_choosen=s_possible[Indice_u,:]
	write_console("Choosen s: ",s_choosen)
	u_choosen=u[u_possible[Indice_u+1]]
	write_console("Choosen u: ",u_choosen)
	write_console("\n----------------------\n","")
	s_connu=s_choosen.'
	s_enreg[t,:]=s_connu
	#=write_console("s_enreg: ","")
	for i=1:size(s_enreg,1)
		write_console(s_enreg[i,:],"")
	end=#
	#write_console(norm(s_enreg[t,1:2].'-s_final))
	if(norm(s_enreg[t,1:2].'-s_final)<1)#moins d'un mettre 'lun de lautre ->ok
		t=Number_t_step
		break
	end
	u_possible=[0]

end

write_console("S_trajet: ",s_enreg)
write_console("Obsta: ",Data1.obstacles)
write_console("Destin: ",s_final)
