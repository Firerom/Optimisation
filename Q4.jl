include("optimisation1.jl")

function abs_(v::Variable)
  @defVar(v.m, aux >= 0)
  @addConstraint(v.m, aux >= v)
  @addConstraint(v.m, aux >= -v)
  return aux
end
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

u=pi #test for pi after
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
@variable(m, c10)
@variable(m, c11)
@variable(m, c12)
@variable(m, c13)
@variable(m, c14)
@variable(m, c15)
@variable(m, c16)
@variable(m, c17)
@variable(m, c18)
@variable(m, c19)
@variable(m, c20)
@variable(m, c21)
@variable(m, c22)
@variable(m, c23)
@variable(m, c24)
@variable(m, c25)
@variable(m, c26)
@variable(m, c27)
@variable(m, c28)
@variable(m, c29)
@variable(m, c30)
@variable(m, c31)
@variable(m, c32)
@variable(m, c33)
@variable(m, c34)



#matrices SDP for obstacle constraint
@variable(m, M_obs[1:Nbr_obstacle,1:3,1:3])
#matrix for the Vdot constraint
@variable(m,M[1:8,1:8])
@SDconstraint(m,M<=0)
for n=1:Nbr_obstacle
	for i=1:3
		for j=i:3
			@constraint(m,M_obs[n,i,j]-M_obs[n,j,i]==0)
		end
	end
@SDconstraint(m,M_obs[n,:,:]>=0)
end


# V(s)
# c0 c1 c2 c3   c4   c5  c6  c7  c8  c9   c10 c11  c12  c13 c14   c15  c16   c17
# 1  x  y  th  x^2   xy  y^2 xth yth th^2 x^3 x^2y xy^2 y^3 thx^2 thxy thy^2 th^2x
# c18   c19  c20 c21  c22    c23  c24 c25   c26    c27    c28   c29     c30    c31     c32   c33   c34
# th^2y th^3 x^4 x^3y x^2y^2 xy^3 y^4 thx^3 thx^2y thxy^2 thy^3 th^2x^2 th^2xy th^2y^2 th^3x th^3y th^4

#Initial configuration of the drone
x_i=Data1.start[1];
y_i=Data1.start[2];
theta_i=Data1.start[3];


#V is minimum degree 2 must be verified or we must impose one more constraint
#@constraint(m, -(c7+c8+c9)>=0.001)

#First constraint initially in S_safe
@constraint(m, c34*theta_i^4 + c32*theta_i^3*x_i + c33*theta_i^3*y_i + c19*theta_i^3 + c29*theta_i^2*x_i^2 + c30*theta_i^2*x_i*y_i + c17*theta_i^2*x_i + c31*theta_i^2*y_i^2 + c18*theta_i^2*y_i + c9*theta_i^2 + c25*theta_i*x_i^3 + c26*theta_i*x_i^2*y_i + c14*theta_i*x_i^2 + c27*theta_i*x_i*y_i^2 + c15*theta_i*x_i*y_i + c7*theta_i*x_i + c28*theta_i*y_i^3 + c16*theta_i*y_i^2 + c8*theta_i*y_i + c3*theta_i + c20*x_i^4 + c21*x_i^3*y_i + c10*x_i^3 + c22*x_i^2*y_i^2 + c11*x_i^2*y_i + c4*x_i^2 + c23*x_i*y_i^3 + c12*x_i*y_i^2 + c5*x_i*y_i + c1*x_i + c24*y_i^4 + c13*y_i^3 + c6*y_i^2 + c2*y_i + c0<=b)
#c34*theta_i^4 + c32*theta_i^3*x_i + c33*theta_i^3*y_i + c19*theta_i^3 + c29*theta_i^2*x_i^2 + c30*theta_i^2*x_i*y_i + c17*theta_i^2*x_i + c31*theta_i^2*y_i^2 + c18*theta_i^2*y_i + c9*theta_i^2 + c25*theta_i*x_i^3 + c26*theta_i*x_i^2*y_i + c14*theta_i*x_i^2 + c27*theta_i*x_i*y_i^2 + c15*theta_i*x_i*y_i + c7*theta_i*x_i + c28*theta_i*y_i^3 + c16*theta_i*y_i^2 + c8*theta_i*y_i + c3*theta_i + c20*x_i^4 + c21*x_i^3*y_i + c10*x_i^3 + c22*x_i^2*y_i^2 + c11*x_i^2*y_i + c4*x_i^2 + c23*x_i*y_i^3 + c12*x_i*y_i^2 + c5*x_i*y_i + c1*x_i + c24*y_i^4 + c13*y_i^3 + c6*y_i^2 + c2*y_i + c0
#Second constraint with the obstacles (M_obs already SDP), for each of them
for i=1:Nbr_obstacle
	x_o=Data1.obstacles[i][1]
	y_o=Data1.obstacles[i][2]
	@constraint(m, M_obs[i,1,1]==c20*x_o^4 + c21*x_o^3*y_o + c10*x_o^3 + c22*x_o^2*y_o^2 + c11*x_o^2*y_o + c4*x_o^2 + c23*x_o*y_o^3 + c12*x_o*y_o^2 + c5*x_o*y_o + c1*x_o + c24*y_o^4 + c13*y_o^3 + c6*y_o^2 + c2*y_o + c0
 -b-small_epsilon)
	@constraint(m, 2*M_obs[i,1,2]==c25*x_o^3 + c26*x_o^2*y_o + c14*x_o^2 + c27*x_o*y_o^2 + c15*x_o*y_o + c7*x_o + c28*y_o^3 + c16*y_o^2 + c8*y_o + c3
 )
	@constraint(m, M_obs[i,2,2]+2*M_obs[i,1,3]==c29*x_o^2 + c30*x_o*y_o + c17*x_o + c31*y_o^2 + c18*y_o + c9)
	@constraint(m, 2*M_obs[i,2,3]==c19 + c32*x_o + c33*y_o)
	@constraint(m, M_obs[i,3,3]==c34)

end
#third constraint V_dot

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
