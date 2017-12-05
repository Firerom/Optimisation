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

function obstacle_near(fichier,s_actual)
    Data1=loadDataFromFile(fichier)
    Rayon=20    #m
    obstacles_new=[0 0]
    for i=1:1:Nbr_obstacle
        x=Data1.obstacles[i][1]
    	y=Data1.obstacles[i][2]
        println(x)
        println(y)
        value=(x-s_actual[1])^2+(y-s_actual[2])^2<=Rayon^2
        println(value)
        if((x-s_actual[1])^2+(y-s_actual[2])^2<=Rayon^2)
            obstacles_new=[obstacles_new; x y]
        end
    end
    return obstacles_new[2:size(obstacles_new,1),:]
end
