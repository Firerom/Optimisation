include("optimisation1.jl")
fichier="obs_behind_side"
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

m = Model(solver=MosekSolver())

@variable(m, c0)
@variable(m, c1)
@variable(m, c2)
@variable(m, c3)
@variable(m, c4)
@variable(m, c5)
@variable(m, c6)
@variable(m, c7)
@variable(m, c8)
@variable(m, c9)
#matrices SDP for obstacle constraint
@variable(m, M_obs[1:Nbr_obstacle,1:2,1:2])
#matrix for the Vdot constraint
@variable(m,M[1:12,1:12])
#variable de stockage pour le calcul de la norm (on a besoin d'un vecteur de taille 2  pour les Nbr_obstacle 2X2 matrix)
@variable(m,Ab[1:Nbr_obstacle,1:2])
#variable de stockage pour le calcul de la norm (on a besoin d'un vecteur de taille 2 ) pour chaque 2X2 matrix ! ça donne 12X12 vector
@variable(m,Ab_dot[1:12,1:12,1:2])
#@SDconstraint(m,M<=0)

for n=1:Nbr_obstacle
	for i=1:2
		for j=i:2
			@constraint(m,M_obs[n,i,j]-M_obs[n,j,i]==0)
		end
		#element diagonaux supéreiur à zero
		@constraint(m,M_obs[n,i,i]>=0)
	end
	@constraint(m,Ab[n,1]-2*M_obs[n,1,2]==0)
	@constraint(m,Ab[n,2]-(M_obs[n,1,1]-M_obs[n,2,2])==0)
	@constraint(m,norm(Ab[n,:])  <=(M_obs[n,1,1]+M_obs[n,2,2]))
end


###
# WARNING
#the matrix should be semi-definite neagtive
#
###
for i=1:12
	for j=i:12
		@constraint(m,M[i,j]-M[j,i]==0)
		@constraint(m,Ab_dot[i,j,1]-2*M[i,j]==0)
		@constraint(m,Ab_dot[i,j,2]-(M[i,i]-M[j,j])==0)
		@constraint(m,norm(Ab_dot[i,j,:])  <=-(M[i,i]+M[j,j]) )
	end
	#element diagonaux inférieur à zero
	@constraint(m,M[i,i]<=0)
end
#V(s) polynome
#c0, c1, c2,    c3, c4,     c5,     c6,  c7,  c8,      c9
# 1,  x,  y, theta, xy, xtheta, ytheta, x^2, y^2, theta^2

#Initial configuration of the drone
x_i=Data1.start[1];
y_i=Data1.start[2];
theta_i=Data1.start[3];


#V is minimum degree 2 must be verified or we must impose one more constraint
#@constraint(m, -(c7+c8+c9)>=0.001)

#First constraint initially in S_safe
@constraint(m, c0+c1*x_i+c2*y_i+c3*theta_i+c4*x_i*y_i+c5*x_i*theta_i+c6*y_i*theta_i+c7*x_i^2+c8*y_i^2+c9*theta_i^2<=b)

#Second constraint with the obstacles (M_obs already SDP), for each of them
for i=1:Nbr_obstacle
	x_o=Data1.obstacles[i][1]
	y_o=Data1.obstacles[i][2]
	@constraint(m, M_obs[i,1,1]==c0+c1*x_o+c2*y_o+c4*x_o*y_o+c7*x_o^2+c8*y_o^2-b-small_epsilon)
	@constraint(m, 2*M_obs[i,1,2]==c3+c5*x_o+c6*y_o)
	@constraint(m, M_obs[i,2,2]==c9)
end

#third constraint V_dot
# 1
@constraint(m,c2*v + K*c3*u==M[1,1])
# x
@constraint(m,c4*v + K*c5*u==2*M[1,2])
# y
@constraint(m,2*c8*v + K*c6*u==2*M[1,3])
# xt
@constraint(m,- K*c5 - 2*c7*v==2*M[2,4])
# xt2
@constraint(m,-(c4*v)/2==2*M[2,5])
# xt3
@constraint(m,(2*c7*v)/6==2*M[2,6])
# xt4
@constraint(m,(c4*v)/24==2*M[2,7])
# xt5
@constraint(m,-(2*c7*v)/120==2*M[2,8])
# xt6
@constraint(m,-(c4*v)/720==2*M[2,9])
# xt7
@constraint(m,(2*c7*v)/5040==2*M[2,10])
# xt9
@constraint(m,-(2*c7*v)/362880==2*M[2,12])
# yt
@constraint(m,- K*c6 - c4*v==2*M[3,4])
# yt2
@constraint(m,-(2*c8*v)/2==2*M[3,5])
# yt3
@constraint(m,(c4*v)/6==2*M[3,6])
# yt4
@constraint(m,(2*c8*v)/24==2*M[3,7])
# yt5
@constraint(m,-(c4*v)/120==2*M[3,8])
# yt6
@constraint(m,-(2*c8*v)/720==2*M[3,9])
# yt7
@constraint(m,(c4*v)/5040==2*M[3,10])
# yt9
@constraint(m,-(c4*v)/362880==2*M[3,12])


# t
@constraint(m,c6*v - c1*v - K*c3 + 2*K*c9*u==2*M[1,4])
# t1
@constraint(m,- 2*K*c9 - c5*v - (c2*v)/2==2*M[1,5]+M[4,4])
# t2
@constraint(m,(c1*v)/6 - (c6*v)/2==2*M[1,6]+M[4,5]+M[4,5])
# t3
@constraint(m,(c2*v)/24 + (c5*v)/6==2*M[1,7]+M[4,6]+M[5,5]+M[4,6])
# t4
@constraint(m,(c6*v)/24 - (c1*v)/120==2*M[1,8]+M[4,7]+M[5,6]+M[5,6]+M[4,7])
# t5
@constraint(m,- (c2*v)/720 - (c5*v)/120==2*M[1,9]+M[4,8]+M[5,7]+M[6,6]+M[5,7]+M[4,8])
# t6
@constraint(m,(c1*v)/5040 - (c6*v)/720==2*M[1,10]+M[4,9]+M[5,8]+M[6,7]+M[6,7]+M[5,8]+M[4,9])
# t7
@constraint(m,(c5*v)/5040==2*M[1,11]+M[4,10]+M[5,9]+M[6,8]+M[7,7]+M[6,8]+M[5,9]+M[4,10])
# t8
@constraint(m,-(c1*v)/362880==2*M[1,12]+M[4,11]+M[5,10]+M[6,9]+M[7,8]+M[7,8]+M[6,9]+M[5,10]+M[4,11])
# t9
@constraint(m,-(c5*v)/362880==M[4,12]+M[5,11]+M[6,10]+M[7,9]+M[8,8]+M[7,9]+M[6,10]+M[5,11]+M[4,12])




# yt8: 0
@constraint(m,0==M[2,11])
# xt8: 0
@constraint(m,0==M[3,11])
# x2: 0
@constraint(m,0==M[2,2])
# y2: 0
@constraint(m,0==M[3,3])
# xy: 0
@constraint(m,0==2*M[2,3])


#theta11:0
@constraint(m,M[5,12]+M[6,11]+M[7,10]+M[8,9]+M[8,9]+M[7,10]+M[6,11]+M[5,12]==0)
#theta12:0
@constraint(m,M[6,12]+M[7,11]+M[8,10]+M[9,9]+M[8,10]+M[7,11]+M[6,12]==0)
#theta13:0
@constraint(m,M[7,12]+M[8,11]+M[9,10]+M[9,10]+M[8,11]+M[7,12]==0)
#theta14:0
@constraint(m,M[8,12]+M[9,11]+M[10,10]+M[9,11]+M[8,12]==0)
#theta15:0
@constraint(m,M[9,12]+M[10,11]+M[10,11]+M[9,12]==0)
#theta16:0
@constraint(m,M[10,12]+M[11,11]+M[10,12]==0)
#theta17:0
@constraint(m,M[11,12]+M[11,12]==0)
#theta18:0
@constraint(m,M[12,12]==0)

solve(m)
println("\n")
println("Les coefficients:")
println(getvalue(c0))
println(getvalue(c1))
println(getvalue(c2))
println(getvalue(c3))
println(getvalue(c4))
println(getvalue(c5))
println(getvalue(c6))
# verify (c7~=0 or c8~=0 or c9~=0)==1
println(getvalue(c7))
println(getvalue(c8))
println(getvalue(c9),"\n")

for n=1:Nbr_obstacle
	for i=1:2
		for j=1:2
			println(getvalue(M_obs[n,i,j]))
		end
	end
end

if u==pi
	ustring="pi"
else
	ustring="0"
end
file_name=string("Q2_",ustring,"_",fichier,".txt")
ci=getvalue([c0 c1 c2 c3 c4 c5 c6 c7 c8 c9])
writedlm(file_name, ci)
;
