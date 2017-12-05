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
#grad_V=zeros(1,3)
s_itera[1,1]=x_i
s_itera[1,2]=y_i
s_itera[1,3]=theta_i
k=0
#X_vec=[1,s_itera[k+1,1],s_itera[k+1,2],s_itera[k+1,3],s_itera[k+1,3]^2,s_itera[k+1,3]^3,s_itera[k+1,3]^4,s_itera[k+1,3]^5,s_itera[k+1,3]^6,s_itera[k+1,3]^7,s_itera[k+1,3]^8,s_itera[k+1,3]^9]
#@constraint(m,<=0)
# compute the trajectories-> add constraint for all other trajectories
#for k=1:Number_t_step
k=1
while(k<Number_t_step+1)
s_dot=[-v*sin(s_itera[k,3])+w,v*cos(s_itera[k,3]),-K*(s_itera[k,3]-u)]
s_itera[k+1,1]=s_itera[k,1]+Time_step*s_dot[1]
s_itera[k+1,2]=s_itera[k,2]+Time_step*s_dot[2]
s_itera[k+1,3]=s_itera[k,3]+Time_step*s_dot[3]
grad_V=[(c1+c4*s_itera[k+1,2]+c5*s_itera[k+1,3]+2*c7*s_itera[k+1,1]),(c2+c4*s_itera[k+1,1]+c6*s_itera[k+1,3]+2*c8*s_itera[k+1,2]),(c3+c5*s_itera[k+1,1]+c6*s_itera[k+1,2]+2*c9*s_itera[k+1,3])]
#X_vec=[1,s_itera[k+1,1],s_itera[k+1,2],s_itera[k+1,3],s_itera[k+1,3]^2,s_itera[k+1,3]^3,s_itera[k+1,3]^4,s_itera[k+1,3]^5,s_itera[k+1,3]^6,s_itera[k+1,3]^7,s_itera[k+1,3]^8,s_itera[k+1,3]^9]
@constraint(m,dot(grad_V,s_dot)<=0)

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
