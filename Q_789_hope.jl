include("optimisation1.jl")

fichier="Romain"

# if load from a file
Data1=loadDataFromFile(fichier)
saveDataToFileTXT(Data1,fichier)

# if generateData of n obstacle
#Data1=generateData(30)
#saveDataToFileTXT(Data1,fichier)
#saveDataToFile(Data1,fichier)

tolerance=0.00
small_epsilon=0.000001


#le nombre d'obstacle
Nbr_obstacle=size(Data1.obstacles,1)
clear_console()

v=6
K=0.2
b=0
w=0
Time_of_fligth=20
Time_step=0.1
Number_t_step=2300
Time_of_fligth=Time_step*Number_t_step
#L=obstacle_near(fichier,[70 90])
k=21
u=-pi:(2*pi/(k-1)):pi
Current_time=0
u_possible=zeros(k)
s_suivant=zeros(k,3)
s_suivantSDP=zeros(k)
s_final=[Data1.destination[1] Data1.destination[2]]
s_connu=[Data1.start[1] Data1.start[2] Data1.start[3]]
s_enreg=zeros(Number_t_step,3)
t=1
while t<=Number_t_step
	write_console("t: ",t)
	u_possible=0
	Current_time+=Time_step
	s_actual=s_connu
	mes_obstacles_near=obstacle_near(fichier,s_actual)

	mes_obstacles=mes_obstacles_near[1]
	number_obst_near=mes_obstacles_near[2]
	write_console("number_obst_near: ",number_obst_near)
	write_console("mes_obstacles: ",mes_obstacles)
	if(number_obst_near!=0)# il y a des obstacles
	#phase avec le calcul de SDP pour V  car obstacle
		nbr_iteration=18
		s_garder=zeros(k,nbr_iteration,3)
		for i_u=1:1:k
			i_u=round(i_u)

			s_temp=s_actual
			for i=1:1:nbr_iteration
				s_dot=[-v*sin(s_temp[3])+w v*cos(s_temp[3]) -K*(s_temp[3]-u[i_u])]
				s_suivantSDP=s_temp+Time_step*s_dot
				s_temp=s_suivantSDP
				s_garder[i_u,i,:]=s_suivantSDP
			end
			s_suivant[i_u,:]=s_suivantSDP
			C=SDP_barrier(mes_obstacles,s_actual,s_suivantSDP,u[i_u],w,Time_step)
			write_console("Coeff: \n",C)
			if(!isnan(C[1]))
				u_possible=[u_possible i_u]
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

	s_connu=s_choosen.'
	s_enreg[t,:]=s_connu
	if(number_obst_near!=0)
		write_console("Indice_u: ",Indice_u)
		write_console("s_garder: ",s_garder[Indice_u,:,:])
		for i=1:1:nbr_iteration
			s_enreg[t,:]=s_garder[Indice_u,i,:]
			t+=1
		end
		t-=1
		write_console("t: ",t)
	end
	#=write_console("s_enreg: ","")
	for i=1:size(s_enreg,1)
		write_console(s_enreg[i,:],"")
	end=#
	#write_console(norm(s_enreg[t,1:2].'-s_final))
	write_console("\n----------------------\n","")
	if(norm(s_enreg[t,1:2].'-s_final)<1)#moins d'un mettre 'lun de lautre ->ok
		t=Number_t_step
		break
	end
	u_possible=[0]
	t+=1
end

write_console("S_trajet: ",s_enreg)
write_console("Obsta: ",Data1.obstacles)
write_console("Destin: ",s_final)
savetrajectoryToFileTXT(s_enreg,fichier)
