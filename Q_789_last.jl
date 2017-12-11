include("optimisation1.jl")

fichier="Romain"

# if load from a file
Data1=loadDataFromFile(fichier)
saveDataToFileTXT(Data1,fichier)

# if generateData of n obstacle
#Data1=generateData(100)
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
w=2
Time_of_fligth=20
Time_step=0.1
Number_t_step=600
Time_of_fligth=Time_step*Number_t_step
#L=obstacle_near(fichier,[70 90])
k=21
nbr_iteration_SDP=20
nbr_iteration_free=2

u=-pi:(2*pi/(k-1)):pi
Current_time=0


u_possible=zeros(k)
s_suivant=zeros(k,3)

s_suivantSDP=zeros(3)
s_suivantfree=zeros(3)

s_final=[Data1.destination[1] Data1.destination[2]]
s_connu=[Data1.start[1] Data1.start[2] Data1.start[3]]
s_enreg=zeros(Number_t_step,3)
s_enreg_final=zeros(Number_t_step,3)
t=1

#distSDP=zeros(nbr_iteration_SDP,1)
#distfree=zeros(nbr_iteration_free,1)
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

		s_garderSDP=zeros(k,nbr_iteration_SDP,3)
		for i_u=1:k
			i_u=round(i_u)

			s_temp=s_actual
			for i=1:nbr_iteration_SDP
				s_dot=[-v*sin(s_temp[3])+w v*cos(s_temp[3]) -K*(s_temp[3]-u[i_u])]
				s_suivantSDP=s_temp+Time_step*s_dot
				s_temp=s_suivantSDP
				s_garderSDP[i_u,i,:]=s_suivantSDP
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
		s_garderfree=zeros(k,nbr_iteration_free,3)
		for i_u=1:1:k
			i_u=round(i_u)
			s_temp=s_actual
			for i=1:nbr_iteration_free
				s_dot=[-v*sin(s_temp[3])+w v*cos(s_temp[3]) -K*(s_temp[3]-u[i_u])]
				s_suivantfree=s_temp+Time_step*s_dot
				s_temp=s_suivantfree
				s_garderfree[i_u,i,:]=s_suivantfree
			end
			s_suivant[i_u,:]=s_suivantfree
			u_possible=[u_possible i_u]
		end

	end

	#calule de meilleur u->joris
	write_console("u possible: ",u_possible)
	s_possible=s_suivant[u_possible[2:length(u_possible)],:]
 	write_console("s_suivant possible: ","")
	for i=1:size(s_possible,1)
		write_console(s_possible[i,:],"")
	end
	#write_console("\nS suivant(dynamique deja fixee): ",[s_possible[1,1] s_possible[1,2]])
	#write_console("S Final: ",s_final,"\narctg a realiser\n")

	if(number_obst_near!=0)
		distSDP=zeros(size(s_possible,1),1)
		for i=1:size(s_possible,1)
			distSDP[i]=norm(s_possible[i,1:2].'-s_final)

		end
		write_console("distSDP= ",distSDP)
		optimum=extrema(distSDP)
		min=optimum[1]
		Indice_u=find_index(distSDP,min)
		#write_console("Indice_u= ",Indice_u)
	else
		distfree=zeros(size(s_possible,1),1)
		for i=1:size(s_possible,1)
			distfree[i]=norm(s_possible[i,1:2].'-s_final)

		end
		write_console("distfree= ",distfree)
		optimum=extrema(distfree)
		min=optimum[1]
		Indice_u=find_index(distfree,min)
	end

	write_console("Choosen index: ",Indice_u)
	s_choosen=s_possible[Indice_u,:]
	write_console("Choosen s: ",s_choosen)
	u_choosen=u[u_possible[Indice_u+1]]
	write_console("Choosen u: ",u_choosen)

	s_connu=s_choosen.'
	#s_enreg[t,:]=s_connu
	if(number_obst_near!=0)
		write_console("Indice_u: ",Indice_u)
		write_console("s_garder: ",s_garderSDP[u_possible[Indice_u+1],:,:])
		write_console("s_choosen: ",s_choosen.')
		count=0
		for i=1:nbr_iteration_SDP
			s_enreg[t,:]=s_garderSDP[u_possible[Indice_u+1],i,:]
			t+=1
			count+=1
			if t>Number_t_step
				break
			end
		end
		#if t>Number_t_step
		#	break
		#end

		write_console("t: ",t)
	else
		write_console("Indice_u: ",Indice_u)
		write_console("s_garder: ",s_garderfree[u_possible[Indice_u+1],:,:])
		write_console("Indice s_garderfree:",u_possible[Indice_u+1])
		write_console("s_choosen: ",s_choosen.')
		count=0
		for i=1:nbr_iteration_free
			s_enreg[t,:]=s_garderfree[u_possible[Indice_u+1],i,:]
			t+=1
			count+=1
			if t>Number_t_step
				break
			end
		end
		#if t>Number_t_step
		#	break
		#end

		write_console("t: ",t)
	end
	#=write_console("s_enreg: ","")
	for i=1:size(s_enreg,1)
		write_console(s_enreg[i,:],"")
	end=#
	#write_console(norm(s_enreg[t,1:2].'-s_final))
	if(t>Number_t_step)
		#write_console("S_trajet: ",s_enreg)
		s_enreg_final=s_enreg
		break
	end

	for i=1:count
		#write_console("",i)
		if(norm(s_enreg[t+i-count,1:2].'-s_final)<1)#moins d'un mettre 'lun de lautre ->ok
			s_enreg_final=s_enreg[1:(t+i-count),1:3]
			t=Number_t_step+1
			break
		end

	end
	if(t>Number_t_step)
		#write_console("S_trajet: ",s_enreg)
		break
	end

	u_possible=[0]
write_console("\n----------------------\n","")
end

write_console("S_trajet: ",s_enreg)
write_console("Obsta: ",Data1.obstacles)
write_console("Destin: ",s_final)
write_console("prout",size(s_enreg_final))
savetrajectoryToFileTXT(s_enreg_final,fichier)
