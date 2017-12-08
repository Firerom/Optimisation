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
	counter=0
	Nbr_obstacle=size(Data1.obstacles[:][1],1)
    for i=1:Nbr_obstacle
        x=Data1.obstacles[i][1]
    	y=Data1.obstacles[i][2]
        #println(x)
        #println(y)
        value=(x-s_actual[1])^2+(y-s_actual[2])^2<=Rayon^2
        #println(value)
        if((x-s_actual[1])^2+(y-s_actual[2])^2<=Rayon^2)
            obstacles_new=[obstacles_new; x y]
			counter+=1
        end
    end
    return obstacles_new[2:size(obstacles_new,1),:] , counter
end

function SDP_barrier(obstacle,s_actual,u)


    small_epsilon=0.000001

    #le nombre d'obstacle
    Nbr_obstacle=size(obstacle,1)

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
	println(nbr)
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



function SDP_barrier_test(obstacle,s_actual,u)


    small_epsilon=0.000001

    #le nombre d'obstacle
    Nbr_obstacle=size(obstacle,1)

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

    #Second constraint with the obstacles (M_obs already SDP), for each of them
    for i=1:Nbr_obstacle
		x_o=obstacle[i,1]
		y_o=obstacle[i,2]
    	@constraint(m, M_obs[i,1,1]==c0+c1*x_o+c2*y_o+c4*x_o*y_o+c7*x_o^2+c8*y_o^2-b-small_epsilon)
    	@constraint(m, 2*M_obs[i,1,2]==c3+c5*x_o+c6*y_o)
    	@constraint(m, M_obs[i,2,2]==c9)
    end


	#s_dot=[-v*sin(s_actual[3])+w,v*cos(s_actual[3]),-K*(s_actual[3]-u)]
	#grad_V=[(c1+c4*s_actual[2]+c5*s_actual[3]+2*c7*s_actual[1]),(c2+c4*s_actual[1]+c6*s_actual[3]+2*c8*s_actual[2]),(c3+c5*s_actual[1]+c6*s_actual[2]+2*c9*s_actual[3])]
	#X_vec=[1,s_itera[k+1,1],s_itera[k+1,2],s_itera[k+1,3],s_itera[k+1,3]^2,s_itera[k+1,3]^3,s_itera[k+1,3]^4,s_itera[k+1,3]^5,s_itera[k+1,3]^6,s_itera[k+1,3]^7,s_itera[k+1,3]^8,s_itera[k+1,3]^9]
	#@constraint(m,dot(grad_V,s_dot)<=0)

    solve(m)

    C=getvalue([c0 c1 c2 c3 c4 c5 c6 c7 c8 c9])
    return C
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

function SDP_barrier_jo(obstacle,s_actual,u)
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
	Nbr_obstacle=size(obstacle,1)

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
	@variable(m,M[1:12,1:12], Symmetric)
	@SDconstraint(m,M>=0)

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

	#Second constraint with the obstacles (M_obs already SDP), for each of them
	for i=1:Nbr_obstacle
		x_o=obstacle[i,1]
		y_o=obstacle[i,2]
		@constraint(m, M_obs[i,1,1]==c0+c1*x_o+c2*y_o+c4*x_o*y_o+c7*x_o^2+c8*y_o^2-b-small_epsilon)
		@constraint(m, 2*M_obs[i,1,2]==c3+c5*x_o+c6*y_o)
		@constraint(m, M_obs[i,2,2]==c9)
	end

	#third constraint V_dot

	# 1
	@constraint(m,c2*v + K*c3*u==-(M[1,1]))
	# x
	@constraint(m,c4*v + K*c5*u==-(2*M[1,2]))
	# y
	@constraint(m,2*c8*v + K*c6*u==-(2*M[1,3]))
	# xt
	@constraint(m,- K*c5 - 2*c7*v==-(2*M[2,4]))
	# xt2
	@constraint(m,-(c4*v)/2==-(2*M[2,5]))
	# xt3
	@constraint(m,(2*c7*v)/6==-(2*M[2,6]))
	# xt4
	@constraint(m,(c4*v)/24==-(2*M[2,7]))
	# xt5
	@constraint(m,-(2*c7*v)/120==-(2*M[2,8]))
	# xt6
	@constraint(m,-(c4*v)/720==-(2*M[2,9]))
	# xt7
	@constraint(m,(2*c7*v)/5040==-(2*M[2,10]))
	# xt9
	@constraint(m,-(2*c7*v)/362880==-(2*M[2,12]))
	# yt
	@constraint(m,- K*c6 - c4*v==-(2*M[3,4]))
	# yt2
	@constraint(m,-(2*c8*v)/2==-(2*M[3,5]))
	# yt3
	@constraint(m,(c4*v)/6==-(2*M[3,6]))
	# yt4
	@constraint(m,(2*c8*v)/24==-(2*M[3,7]))
	# yt5
	@constraint(m,-(c4*v)/120==-(2*M[3,8]))
	# yt6
	@constraint(m,-(2*c8*v)/720==-(2*M[3,9]))
	# yt7
	@constraint(m,(c4*v)/5040==-(2*M[3,10]))
	# yt9
	@constraint(m,-(c4*v)/362880==-(2*M[3,12]))


	# t
	@constraint(m,c6*v - c1*v - K*c3 + 2*K*c9*u==-(2*M[1,4]))
	# t2
	@constraint(m,- 2*K*c9 - c5*v - (c2*v)/2==-(2*M[1,5]+M[4,4]))
	# t3
	@constraint(m,(c1*v)/6 - (c6*v)/2==-(2*M[1,6]+M[4,5]+M[4,5]))
	# t4
	@constraint(m,(c2*v)/24 + (c5*v)/6==-(2*M[1,7]+M[4,6]+M[5,5]+M[4,6]))
	# t5
	@constraint(m,(c6*v)/24 - (c1*v)/120==-(2*M[1,8]+M[4,7]+M[5,6]+M[5,6]+M[4,7]))
	# t6
	@constraint(m,- (c2*v)/720 - (c5*v)/120==-(2*M[1,9]+M[4,8]+M[5,7]+M[6,6]+M[5,7]+M[4,8]))
	# t7
	@constraint(m,(c1*v)/5040 - (c6*v)/720==-(2*M[1,10]+M[4,9]+M[5,8]+M[6,7]+M[6,7]+M[5,8]+M[4,9]))
	# t8
	@constraint(m,(c5*v)/5040==-(2*M[1,11]+M[4,10]+M[5,9]+M[6,8]+M[7,7]+M[6,8]+M[5,9]+M[4,10]))
	# t9
	@constraint(m,-(c1*v)/362880==-(2*M[1,12]+M[4,11]+M[5,10]+M[6,9]+M[7,8]+M[7,8]+M[6,9]+M[5,10]+M[4,11]))
	# t10
	@constraint(m,-(c5*v)/362880==-(M[4,12]+M[5,11]+M[6,10]+M[7,9]+M[8,8]+M[7,9]+M[6,10]+M[5,11]+M[4,12]))




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

	C=getvalue([c0 c1 c2 c3 c4 c5 c6 c7 c8 c9])
    return C
end
