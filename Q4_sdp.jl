include("optimisation1.jl")


fichier="obs_behind_side"
Data1=loadDataFromFile(fichier)
small_epsilon=0.000001
objectif_without=-999999
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
w=0
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
@variable(m,M[1:8,1:8], Symmetric)
#matrix fro the V_s sontaint
@variable(m, M_s[1:3,1:3], Symmetric)
@variable(m,h)
if abs(objectif_without)<1e-5
	@objective(m, Min, h)
else
	h=objectif_without
end
#h=-9999999999999

@SDconstraint(m,M>=0)
for n=1:Nbr_obstacle
	for i=1:3
		for j=i:3
			@constraint(m,M_obs[n,i,j]-M_obs[n,j,i]==0)
		end
	end
@SDconstraint(m,M_obs[n,:,:]>=0)
end
@SDconstraint(m,M_s>=0)

# V(s)
# c0 c1 c2 c3   c4   c5  c6  c7  c8  c9   c10 c11  c12  c13 c14   c15  c16   c17
# 1  x  y  th  x^2   xy  y^2 xth yth th^2 x^3 x^2y xy^2 y^3 thx^2 thxy thy^2 th^2x
# c18   c19  c20 c21  c22    c23  c24 c25   c26    c27    c28   c29     c30    c31     c32   c33   c34
# th^2y th^3 x^4 x^3y x^2y^2 xy^3 y^4 thx^3 thx^2y thxy^2 thy^3 th^2x^2 th^2xy th^2y^2 th^3x th^3y th^4

#Initial configuration of the drone
x_i=Data1.start[1];
y_i=Data1.start[2];
theta_i=Data1.start[3];


#V is minimum degree 5 must be verified or we must impose one more constraint
#@constraint(m, -(c7+c8+c9)>=0.001)

#First constraint initially in S_safe
@constraint(m, c34*theta_i^4 + c32*theta_i^3*x_i + c33*theta_i^3*y_i + c19*theta_i^3 + c29*theta_i^2*x_i^2 + c30*theta_i^2*x_i*y_i + c17*theta_i^2*x_i + c31*theta_i^2*y_i^2 + c18*theta_i^2*y_i + c9*theta_i^2 + c25*theta_i*x_i^3
+ c26*theta_i*x_i^2*y_i + c14*theta_i*x_i^2 + c27*theta_i*x_i*y_i^2 + c15*theta_i*x_i*y_i + c7*theta_i*x_i + c28*theta_i*y_i^3 + c16*theta_i*y_i^2 + c8*theta_i*y_i + c3*theta_i + c20*x_i^4 + c21*x_i^3*y_i + c10*x_i^3 + c22*x_i^2*y_i^2
+ c11*x_i^2*y_i + c4*x_i^2 + c23*x_i*y_i^3 + c12*x_i*y_i^2 + c5*x_i*y_i + c1*x_i + c24*y_i^4 + c13*y_i^3 + c6*y_i^2 + c2*y_i + c0<=b-small_epsilon)
#Second constraint with the obstacles (M_obs already SDP), for each of them
for i=1:Nbr_obstacle
	x_o=Data1.obstacles[i][1]
	y_o=Data1.obstacles[i][2]
	@constraint(m, M_obs[i,1,1]==c20*x_o^4 + c21*x_o^3*y_o + c10*x_o^3 + c22*x_o^2*y_o^2 + c11*x_o^2*y_o + c4*x_o^2 + c23*x_o*y_o^3 + c12*x_o*y_o^2 + c5*x_o*y_o + c1*x_o + c24*y_o^4
	+ c13*y_o^3 + c6*y_o^2 + c2*y_o + c0-b-small_epsilon)
	@constraint(m, 2*M_obs[i,1,2]==c25*x_o^3 + c26*x_o^2*y_o + c14*x_o^2 + c27*x_o*y_o^2 + c15*x_o*y_o + c7*x_o + c28*y_o^3 + c16*y_o^2 + c8*y_o + c3)
	@constraint(m, M_obs[i,2,2]+2*M_obs[i,1,3]==c29*x_o^2 + c30*x_o*y_o + c17*x_o + c31*y_o^2 + c18*y_o + c9)
	@constraint(m, 2*M_obs[i,2,3]==c19 + c32*x_o + c33*y_o)
	@constraint(m, M_obs[i,3,3]==c34)

end

#third constraint V_dot
#1
@constraint(m, M[1,1]==-(c2*v + c1*w + K*c3*u))
#x
@constraint(m, 2*M[1,2]==-(c5*v + 2*c4*w + K*c7*u))
#y
@constraint(m, 2*M[1,3]==-(2*c6*v + c5*w + K*c8*u))
#t
@constraint(m, 2*M[1,4]==-(c8*v + c7*w + 2*K*c9*u))
#x^2
@constraint(m, 2*M[1,6]+M[2,2]==-(c11*v + 3*c10*w + K*c14*u))
#y^2
@constraint(m, 2*M[1,7]+M[3,3]==  -(3*c13*v + c12*w + K*c16*u))
#t^2
@constraint(m, 2*M[1,8]+M[4,4]== -(c18*v + c17*w + 3*K*c19*u))
#x*y
@constraint(m, 2*M[1,5]+2*M[2,3]== -(2*c12*v + 2*c11*w + K*c15*u))
#t*x
@constraint(m, 2*M[2,4]== -(c15*v + 2*c14*w + 2*K*c17*u))
#t*y
@constraint(m, 2*M[3,4]==-(2*c16*v + c15*w + 2*K*c18*u))
#x^3
@constraint(m, 2*M[2,6]== -(c21*v + 4*c20*w + K*c25*u))
#y^3
@constraint(m, 2*M[3,7]==-(4*c24*v + c23*w + K*c28*u))
#x*y^2
@constraint(m, 2*M[2,7]+M[3,5]== -(3*c23*v + 2*c22*w + K*c27*u))
#t^2*x
@constraint(m, 2*M[2,8]==-(c30*v + 2*c29*w + 3*K*c32*u))
#x^2*y
@constraint(m, 2*M[2,5]+M[3,6]== -(2*c22*v + 3*c21*w + K*c26*u))
#t^2*y
@constraint(m, 2*M[3,8]==-(2*c31*v + c30*w + 3*K*c33*u))
#t^3
@constraint(m, 2*M[4,8]== -(c33*v + c32*w + 4*K*c34*u))
#t*x^2
@constraint(m, 2*M[4,6]== -(c26*v + 3*c25*w + 2*K*c29*u))
#t*y^2
@constraint(m, 2*M[4,7]==-(3*c28*v + c27*w + 2*K*c31*u))
#t*x*y
@constraint(m, 2*M[4,5]== -(2*c27*v + 2*c26*w + 2*K*c30*u))
#x4
@constraint(m, M[6,6]==0)
#y^4
@constraint(m, M[7,7]==0)
#t^4
@constraint(m, M[8,8]==0)
#x^2*y^2
@constraint(m, M[5,5]+2*M[6,7]==0)
#t^2*x^2
@constraint(m, 2*M[6,8]==0)
#t^2*y^2
@constraint(m, 2*M[7,8]==0)
#t^2*x*y
@constraint(m, 2*M[5,8]==0)
#x^3*y
@constraint(m, 2*M[5,6]==0)
#x*y^3
@constraint(m, 2*M[5,7]==0)

#fourth constraint V(S_final)<h
x_o=Data1.destination[1]
y_o=Data1.destination[2]
@constraint(m, M_s[1,1]==-(c20*x_o^4 + c21*x_o^3*y_o + c10*x_o^3 + c22*x_o^2*y_o^2 + c11*x_o^2*y_o + c4*x_o^2 + c23*x_o*y_o^3 + c12*x_o*y_o^2 + c5*x_o*y_o + c1*x_o + c24*y_o^4
+ c13*y_o^3 + c6*y_o^2 + c2*y_o + c0-h-small_epsilon))
@constraint(m, 2*M_s[1,2]==-(c25*x_o^3 + c26*x_o^2*y_o + c14*x_o^2 + c27*x_o*y_o^2 + c15*x_o*y_o + c7*x_o + c28*y_o^3 + c16*y_o^2 + c8*y_o + c3))
@constraint(m, M_s[2,2]+2*M_s[1,3]==-(c29*x_o^2 + c30*x_o*y_o + c17*x_o + c31*y_o^2 + c18*y_o + c9))
@constraint(m, 2*M_s[2,3]==-(c19 + c32*x_o + c33*y_o))
@constraint(m, M_s[3,3]==-c34)



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
	for i=1:3
		for j=1:3
			println(getvalue(M_obs[n,i,j]))
		end
	end
end

if u==pi
	ustring="pi"
else
	ustring="0"
end
file_name=string("Result_Q4_SDP/Q4_",ustring,"_",fichier,"_",h,".txt")
ci=getvalue([c0 c1 c2 c3   c4   c5  c6  c7  c8  c9   c10 c11  c12  c13 c14   c15  c16   c17 c18   c19  c20 c21  c22    c23  c24 c25   c26    c27    c28   c29     c30    c31     c32   c33   c34])
writedlm(file_name, ci)

println(getvalue([c0 c1 c2 c3   c4   c5  c6  c7  c8  c9   c10 c11  c12  c13 c14   c15  c16   c17 c18   c19  c20 c21  c22    c23  c24 c25   c26    c27    c28   c29     c30    c31     c32   c33   c34]))
println(getvalue(h))

;
