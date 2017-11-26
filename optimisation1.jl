using JLD
using Gurobi
using JuMP
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
#start: (50,100,0) [x,y,theta]
#destination: (50,190) [x,y]
#obstacle: x_obstacle1=Data1.obstacles[1][1]
#obstacle: y_obstacle1=Data1.obstacles[1][2]
#obstacles are behind: (40,90) et (60,90) [90<100]

m = Model(solver=GurobiSolver())

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
#V(s) polynome
#c0, c1, c2,    c3, c4,     c5,     c6,  c7,  c8,      c9
# 1,  x,  y, theta, xy, xtheta, ytheta, x^2, y^2, theta^2

x_i=Data1.start[1];
y_i=Data1.start[2];
theta_i=Data1.start[3];
#First constraint initially in S_safe
@constraint(m, c0+c1*x_i+c2*y_i+c3*theta_i+c4*x_i*y_i+c5*x_i*theta_i+c6*y_i*theta_i+c7*x_i^2+c8*y_i^2+c9*theta_i^2<=b)


Nbr_obstacle=size(Data1.obstacles,1)
i=1
while i <= Nbr_obstacle
	x_o=Data1.obstacles[i][1]
	y_o=Data1.obstacles[i][2]
	theta_o=0
	#@constraint(m, c0+c1*x_o+c2*y_o+c3*theta_o+c4*x_o*y_o+c5*x_o*theta_o+c6*y_o*theta_o+c7*x_o^2+c8*y_o^2+c9*theta_o^2>b)
	println(c0+c1*x_o+c2*y_o+c3*theta_o+c4*x_o*y_o+c5*x_o*theta_o+c6*y_o*theta_o+c7*x_o^2+c8*y_o^2+c9*theta_o^2)
	i+=1
end



#@SDconstraint(m,M>=0)

#@objective(m, Max, 2*icecream + 7*butter)

#@constraint(m, 3*icecream + 7*butter <= 80)
#@constraint(m, icecream <= 20)
#@constraint(m, 1/15*icecream + butter <= 6)

solve(m)
#println(getvalue(icecream))
;
