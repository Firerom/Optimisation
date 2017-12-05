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

u=3*pi/4
w=0
v=6
K=0.2
b=0
Time_of_fligth=1
Time_step=0.1
Number_t_step=Time_of_fligth/Time_step
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
#@variable(m,M[1:12,1:12], Symmetric)
#@SDconstraint(m,M<=0)
@variable(m,M[1:12,1:12])



#### on peut jouer avec SDP ou pas, pour voir ce que ça fait en plus
#(si SDP, on oblige V_dot<0 partout !)







# each matrix obstacle must be symmetric and SDP
for n=1:Nbr_obstacle
	for i=1:2
		for j=i:2
			@constraint(m,M_obs[n,i,j]-M_obs[n,j,i]==0)
		end
	end
@SDconstraint(m,M_obs[n,:,:]>=0)
end


#V(s) polynome
#c0, c1, c2,    c3, c4,     c5,     c6,  c7,  c8,      c9
# 1,  x,  y, theta, xy, xtheta, ytheta, x^2, y^2, theta^2

#Initial configuration of the drone
x_i=Data1.start[1];
y_i=Data1.start[2];
theta_i=Data1.start[3];




## First time step
#First constraint initially in S_safe
@constraint(m, c0+c1*x_i+c2*y_i+c3*theta_i+c4*x_i*y_i+c5*x_i*theta_i+c6*y_i*theta_i+c7*x_i^2+c8*y_i^2+c9*theta_i^2<=b-small_epsilon)
s_itera=zeros(Number_t_step+1,3)

s_itera[1,1]=x_i
s_itera[1,2]=y_i
s_itera[1,3]=theta_i
k=0
X_vec=[1,s_itera[k+1,1],s_itera[k+1,2],s_itera[k+1,3],s_itera[k+1,3]^2,s_itera[k+1,3]^3,s_itera[k+1,3]^4,s_itera[k+1,3]^5,s_itera[k+1,3]^6,s_itera[k+1,3]^7,s_itera[k+1,3]^8,s_itera[k+1,3]^9]
@constraint(m,X_vec[:].'*M[:,:]*X_vec[:]<=0)
# compute the trajectories-> add constraint for all other trajectories
#for k=1:Number_t_step
k=1
while(k<Number_t_step+1)
s_dot=(-v*sin(s_itera[k,3])+w,v*cos(s_itera[k,3]),-K*(s_itera[k,3]-u))
s_itera[k+1,1]=s_itera[k,1]+Time_step*s_dot[1]
s_itera[k+1,2]=s_itera[k,2]+Time_step*s_dot[2]
s_itera[k+1,3]=s_itera[k,3]+Time_step*s_dot[3]

X_vec=[1,s_itera[k+1,1],s_itera[k+1,2],s_itera[k+1,3],s_itera[k+1,3]^2,s_itera[k+1,3]^3,s_itera[k+1,3]^4,s_itera[k+1,3]^5,s_itera[k+1,3]^6,s_itera[k+1,3]^7,s_itera[k+1,3]^8,s_itera[k+1,3]^9]
@constraint(m,X_vec[:].'*M[:,:]*X_vec[:]<=0)

k=k+1

end


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
@constraint(m,c4*v + K*c5*u==M[1,2]+M[2,1])
# y
@constraint(m,2*c8*v + K*c6*u==M[1,3]+M[3,1])
# xt
@constraint(m,- K*c5 - 2*c7*v==M[2,4]+M[4,2])
# xt2
@constraint(m,-(c4*v)/2==M[2,5]+M[5,2])
# xt3
@constraint(m,(2*c7*v)/6==M[2,6]+M[6,2])
# xt4
@constraint(m,(c4*v)/24==M[2,7]+M[7,2])
# xt5
@constraint(m,-(2*c7*v)/120==M[2,8]+M[8,2])
# xt6
@constraint(m,-(c4*v)/720==M[2,9]+M[9,2])
# xt7
@constraint(m,(2*c7*v)/5040==M[2,10]+M[10,2])
# xt9
@constraint(m,-(2*c7*v)/362880==M[2,12]+M[12,2])
# yt
@constraint(m,- K*c6 - c4*v==M[3,4]+M[3,4])
# yt2
@constraint(m,-(2*c8*v)/2==M[3,5]+M[5,3])
# yt3
@constraint(m,(c4*v)/6==M[3,6]+M[6,3])
# yt4
@constraint(m,(2*c8*v)/24==M[3,7]+M[7,3])
# yt5
@constraint(m,-(c4*v)/120==M[3,8]+M[8,3])
# yt6
@constraint(m,-(2*c8*v)/720==M[3,9]+M[9,3])
# yt7
@constraint(m,(c4*v)/5040==M[3,10]+M[10,3])
# yt9
@constraint(m,-(c4*v)/362880==M[3,12]+M[12,3])


# t
@constraint(m,c6*v - c1*v - K*c3 + 2*K*c9*u==M[1,4]+M[4,1])
# t2
@constraint(m,- 2*K*c9 - c5*v - (c2*v)/2==M[1,5]+M[5,1]+M[4,4])
# t3
@constraint(m,(c1*v)/6 - (c6*v)/2==M[1,6]+M[6,1]+M[4,5]+M[4,5])
# t4
@constraint(m,(c2*v)/24 + (c5*v)/6==M[1,7]+M[7,1]+M[4,6]+M[5,5]+M[4,6])
# t5
@constraint(m,(c6*v)/24 - (c1*v)/120==M[1,8]+M[8,1]+M[4,7]+M[5,6]+M[5,6]+M[4,7])
# t6
@constraint(m,- (c2*v)/720 - (c5*v)/120==M[1,9]+M[9,1]+M[4,8]+M[5,7]+M[6,6]+M[5,7]+M[4,8])
# t7
@constraint(m,(c1*v)/5040 - (c6*v)/720==M[1,10]+M[10,1]+M[4,9]+M[5,8]+M[6,7]+M[6,7]+M[5,8]+M[4,9])
# t8
@constraint(m,(c5*v)/5040==M[1,11]+M[11,1]+M[4,10]+M[5,9]+M[6,8]+M[7,7]+M[6,8]+M[5,9]+M[4,10])
# t9
@constraint(m,-(c1*v)/362880==M[1,12]+M[12,1]+M[4,11]+M[5,10]+M[6,9]+M[7,8]+M[7,8]+M[6,9]+M[5,10]+M[4,11])
# t10
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
@constraint(m,0==M[2,3]+M[3,2])


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
file_name=string("Q1_",ustring,"_",fichier,".txt")
ci=getvalue([c0 c1 c2 c3 c4 c5 c6 c7 c8 c9])
writedlm(file_name, ci)



;
