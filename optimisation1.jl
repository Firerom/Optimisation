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

function saveDataToFileTXT(data::Optinum.Data, instanceName::String)
	fileName = string("Matlabdonnee/Donne_", instanceName, ".txt")
	#save(fileName, "instance", data)
	s_final_1=data.destination[1]
	s_final_2=data.destination[2]
	s_connu_1=data.start[1]
	s_connu_2=data.start[2]
	s_connu_3=data.start[3]
	Nbr_obstacle=size(data.obstacles,1)
	open(fileName, "w") do f
	         write(f, "$s_connu_1 $s_connu_2 $s_connu_3\n")
			 write(f, "$s_final_1 $s_final_2 \n")
			 write(f,"$Nbr_obstacle\n")
			 for i=1:Nbr_obstacle
				 s_obs_1_i=data.obstacles[i][1]
				 s_obs_2_i=data.obstacles[i][2]
				 write(f, "$s_obs_1_i $s_obs_2_i\n")

			 end
	end# n
	println("Data saved to $fileName")
end

function saveDataToFile(data::Optinum.Data, instanceName::String)
	fileName = string("data/", instanceName, ".jld")
	save(fileName, "instance", data)
	println("Data saved to $fileName")
end

function savetrajectoryToFileTXT(s_enreg,instanceName::String)
	fileName = string("Matlabdonnee/trajectory_", instanceName, ".txt")
	length=size(s_enreg,1)
	#println("length=",length)
	open(fileName, "w") do f
		for i=1:length
			s_1=s_enreg[i,1]
			s_2=s_enreg[i,2]
			s_3=s_enreg[i,3]
			 write(f, "$s_1 $s_2 $s_3\n")
		end
	end
	println("trajectory saved to $fileName")
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
	counter=0
	Nbr_obstacle=size(Data1.obstacles,1)
	#println(Data1.obstacles[4][2])
	#println(size(Data1.obstacles,1))
	#println(Nbr_obstacle)
	#println(Data1.obstacles)
    for i=1:Nbr_obstacle
        x=Data1.obstacles[i][1]
    	y=Data1.obstacles[i][2]
        #println(x)
        #println(y)
        value=(x-s_actual[1])^2+(y-s_actual[2])^2
        #println(value)
        if((x-s_actual[1])^2+(y-s_actual[2])^2<=Rayon^2)
            obstacles_new=[obstacles_new; x y]
			counter+=1
        end
    end
    return obstacles_new[2:size(obstacles_new,1),:] , counter
end

function SDP_barrier(obstacle,s_actual,s_suivant,u,w,Time_step)


    small_epsilon=0.001

    #le nombre d'obstacle
    Nbr_obstacle=size(obstacle,1)

    v=6
    K=0.2
    b=0

	#compute a circle around the start(actual) position

	#XY_R=Time_step*s_dot
	#R_max=norm([XY_R[1] XY_R[2]])
	R=2
	k=4
	angle=0:(2*pi/(k)):(2*pi)
	Point=zeros(k,k,2)
	Point_s=zeros(k,k,2)
	for i=1:1:k
		Point[i,1]=R*cos(angle[i])+s_actual[1]
		Point[i,2]=R*sin(angle[i])+s_actual[2]
		Point_s[i,1]=R*cos(angle[i])+s_suivant[1]
		Point_s[i,2]=R*sin(angle[i])+s_suivant[2]
	end


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
    x_i=s_actual[1];
    y_i=s_actual[2];
    theta_i=s_actual[3];


    #V is minimum degree 2 must be verified or we must impose one more constraint
    #@constraint(m, -(c7+c8+c9)>=0.001)

    #First constraint initially in S_safe
    @constraint(m, c0+c1*x_i+c2*y_i+c3*theta_i+c4*x_i*y_i+c5*x_i*theta_i+c6*y_i*theta_i+c7*x_i^2+c8*y_i^2+c9*theta_i^2+small_epsilon<=b)

	for i=1:1:size(Point,1)
		x_i=Point[i,1]
		y_i=Point[i,2]
		theta_i=s_actual[3]
		@constraint(m, c0+c1*x_i+c2*y_i+c3*theta_i+c4*x_i*y_i+c5*x_i*theta_i+c6*y_i*theta_i+c7*x_i^2+c8*y_i^2+c9*theta_i^2+small_epsilon<=b)
		x_i=Point_s[i,1]
		y_i=Point_s[i,2]
		theta_i=s_suivant[3]
		@constraint(m, c0+c1*x_i+c2*y_i+c3*theta_i+c4*x_i*y_i+c5*x_i*theta_i+c6*y_i*theta_i+c7*x_i^2+c8*y_i^2+c9*theta_i^2+small_epsilon<=b)
	end

    #Second constraint with the obstacles (M_obs already SDP), for each of them
    for i=1:Nbr_obstacle
		x_o=obstacle[i,1]
		y_o=obstacle[i,2]
    	@constraint(m, M_obs[i,1,1]==c0+c1*x_o+c2*y_o+c4*x_o*y_o+c7*x_o^2+c8*y_o^2-b-small_epsilon)
    	@constraint(m, 2*M_obs[i,1,2]==c3+c5*x_o+c6*y_o)
    	@constraint(m, M_obs[i,2,2]==c9)
    end


	s_dot=[-v*sin(s_actual[3])+w,v*cos(s_actual[3]),-K*(s_actual[3]-u)]
	grad_V=[(c1+c4*s_actual[2]+c5*s_actual[3]+2*c7*s_actual[1]),(c2+c4*s_actual[1]+c6*s_actual[3]+2*c8*s_actual[2]),(c3+c5*s_actual[1]+c6*s_actual[2]+2*c9*s_actual[3])]
	#X_vec=[1,s_itera[k+1,1],s_itera[k+1,2],s_itera[k+1,3],s_itera[k+1,3]^2,s_itera[k+1,3]^3,s_itera[k+1,3]^4,s_itera[k+1,3]^5,s_itera[k+1,3]^6,s_itera[k+1,3]^7,s_itera[k+1,3]^8,s_itera[k+1,3]^9]
	@constraint(m,dot(grad_V,s_dot)<=0)

	#for i=1:1:k
	#	s_dot1=[-v*sin(s_actual[3])+w,v*cos(s_actual[3]),-K*(s_actual[3]-u)]
	#	grad_V1=[(c1+c4*Point[i,2]+c5*s_actual[3]+2*c7*Point[i,1]),(c2+c4*Point[i,1]+c6*s_actual[3]+2*c8*Point[i,2]),(c3+c5*Point[i,1]+c6*Point[i,2]+2*c9*s_actual[3])]
	#	@constraint(m,dot(grad_V1,s_dot1)<=0)
	#end


    solve(m)

    C=getvalue([c0 c1 c2 c3 c4 c5 c6 c7 c8 c9])
    return C
end

function find_index(matrix,element)
	for i=1:1:length(matrix)
		if abs(abs(matrix[i])-element)<1e-5
			return i
		end
	end
end

function write_console(args...)
	io = open("Console.txt", "a+");
	nbr=size(args,1)
	#println(nbr)
	if nbr==1
		println(io, args);
	elseif nbr==2
		println(io, args[1], args[2]);
	elseif nbr==3
		println(io, args[1], args[2], args[3]);
	else
		println(io,"ATTENTION,MANQUE DE DONNEE")
	end
	close(io);
end

function clear_console()
	io = open("Console.txt", "w");
	println(io, "Begin\n");
	close(io);
end





function mon_arctg(Initial,Final)#compris entre pi et -pi cercle trigo
	alpha=atan((-Initial[2]+Final[2])/(Final[1]-Initial[1]))
	if (-Initial[1]+Final[1])>=0 #cos>0
		alpha=alpha
	else
		if (-Initial[2]+Final[2])>=0 #sin>0
			alpha=pi+alpha
		else
			alpha=alpha-pi
		end
	end
	return alpha
end
