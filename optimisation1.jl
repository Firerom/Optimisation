using JLD
using Gurobi
using JuMP
using Mosek
###################################################

module Optinum
	struct Data
		start::Tuple{Float64,Float64,Float64}
		destination::Tuple{Float64,Float64}
		obstacles::Array{Tuple{Float64,Float64}}
	end
end

###################################################

function loadDataFromFile(instanceName::String)
	fileName = string("data/", instanceName, ".jld")
	return load(fileName, "instance")
end

function saveDataToFile(data::Optinum.Data, instanceName::String)
	fileName = string("data/", instanceName, ".jld")
	save(fileName, "instance", data)
	println("Data saved to $fileName")
end

###################################################

function generateData(numObs::Int)::Optinum.Data
	width = 100 # window width
	height = 200 # window height
	leftX = 0.0 # leftmost x value
	downY = 0.0 # downmost y value
	minObsDistance = 5 # minimum distance so no obstacle is too close to start or destination

	centerX = leftX + width/2.0
	centerY = downY + height/2.0

	start = tuple(centerX, downY+2, 0.0)

	destMiddleBandPercentage = 0.7 # be in the middle 70% of allowed x
	destUpBandPercentage = 0.1 # be in the top 10% of allowed y
	destination = tuple(rand()*width*destMiddleBandPercentage+leftX+(1-destMiddleBandPercentage)/2.0*width,
	                    rand()*height*destUpBandPercentage+downY+(1-destUpBandPercentage)*height)

	# all generated x in [leftX, leftX+width]
	# all generated y in [downY, downY+height]

	obstacles = Array{Tuple{Float64,Float64}}(0)
	numObsRemain = numObs
	while true
		obstaclesNew = [ tuple(rand()*width+leftX, rand()*height+downY) for i in 1:numObsRemain ]
		filter!(t -> norm(collect(t)-collect(start[1:2])) >= minObsDistance &&
		             norm(collect(t)-collect(destination)) >= minObsDistance,
		        obstaclesNew)
		push!(obstacles, obstaclesNew...)

		numObsRemain = numObs - length(obstacles)
		numObsRemain == 0 && break
	end

	return Optinum.Data(start, destination, obstacles)
end

Data1=loadDataFromFile("obs_behind")
small_epsilon=0.000001
#start: (50,100,0) [x,y,theta]
#destination: (50,190) [x,y]
#obstacle: x_obstacle1=Data1.obstacles[1][1]
#obstacle: y_obstacle1=Data1.obstacles[1][2]
#obstacles are behind: (40,90) et (60,90) [90<100]

#le nombre d'obstacle
Nbr_obstacle=size(Data1.obstacles,1)

m = Model(solver=MosekSolver())

@variable(m, b)
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
@variable(m,M[1:12,1:12], Symmetric)
@SDconstraint(m,M<=0)

# matrix obstacle must be symmetric and SDP
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

#V is minimum degree 2 must be verified or we must impose one more constraint
#@constraint(m, -(c7+c8+c9)>=0.001)

#First constraint initially in S_safe
@constraint(m, c0+c1*x_i+c2*y_i+c3*theta_i+c4*x_i*y_i+c5*x_i*theta_i+c6*y_i*theta_i+c7*x_i^2+c8*y_i^2+c9*theta_i^2<=b)

#Second constraint with the obstacles (M_obs already SDP)
for i=1:Nbr_obstacle
	x_o=Data1.obstacles[i][1]
	y_o=Data1.obstacles[i][2]
	@constraint(m, M_obs[i,1,1]==c0+c1*x_o+c2*y_o+c4*x_o*y_o+c7*x_o^2+c8*y_o^2-b-small_epsilon)
	@constraint(m, 2*M_obs[i,1,2]==c3+c5*x_o+c6*y_o)
	@constraint(m, M_obs[i,2,2]==c9)
end

#third constraint V_dot
#theta8:0
@constraint(m,2*M[1,11]+M[4,10]+M[5,9]+M[6,8]+M[7,7]+M[6,8]+M[5,9]+M[4,10]==0)

#theta10:0
@constraint(m,M[4,12]+M[5,11]+M[6,10]+M[7,9]+M[8,8]+M[7,9]+M[6,10]+M[5,11]+M[4,12]==0)
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
println(getvalue(b),"\n")
for n=1:Nbr_obstacle
	for i=1:2
		for j=1:2
			println(getvalue(M_obs[n,i,j]))
		end
	end
end

#@printf("Essai %e",getvalue(b))

;
