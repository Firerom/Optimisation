using JuMP
using CPLEX

const LinConstrRef = ConstraintRef{Model,LinearConstraint}

function getSlacksCplex(c::LinConstrRef)
	if length(c.m.linconstrDuals) != MathProgBase.numlinconstr(c.m)
        error("Sensitivity information not available.")
    end
	solverModel = MathProgBase.getrawsolver(c.m)
	beginIdx = c.idx - 1 # Julia starts indexing at 1, C starts at 0
	endIdx = c.idx - 1
	slack = Array{Cdouble}(1)
	status = ccall((:CPXgetslack, CPLEX.libcplex), Cint, (Ptr{Void}, Ptr{Void}, Ptr{Cdouble}, Cint, Cint), solverModel.env.ptr, solverModel.lp, slack, beginIdx, endIdx)
	if status != 0
		error("CPXgetslack failed with error code $status")
	end
	return slack[1]
end

function getSlacksCplex{DIM}(c::Array{JuMP.ConstraintRef, DIM})
	if length(c[1].m.linconstrDuals) != MathProgBase.numlinconstr(c[1].m)
        error("Sensitivity information not available.")
    end
	numCtr = length(c)
	slacks = Array{Float64}(numCtr)
	for i = 1:numCtr
		# Gotta call the C API for each ctr and not for all of them at once
		# because the index may not be in order or contiguous
		slacks[i] = getSlacksCplex(c[i])
	end
	return slacks
end

function getSlacksCplex{DIM, ANY}(c::JuMP.JuMPArray{JuMP.ConstraintRef, DIM, ANY})
	return getSlacksCplex(c.innerArray)
end

function getBasisBoundsCplex(c::LinConstrRef)
	if length(c.m.linconstrDuals) != MathProgBase.numlinconstr(c.m)
        error("Sensitivity information not available.")
    end
	solverModel = MathProgBase.getrawsolver(c.m)
	beginIdx = c.idx - 1 # Julia starts indexing at 1, C starts at 0
	endIdx = c.idx - 1
	lower = Array{Cdouble}(1)
	upper = Array{Cdouble}(1)
	status = ccall((:CPXrhssa, CPLEX.libcplex), Cint, (Ptr{Void}, Ptr{Void}, Cint, Cint, Ptr{Cdouble}, Ptr{Cdouble}), solverModel.env.ptr, solverModel.lp, beginIdx, endIdx, lower, upper)
	if status != 0
		error("CPXrhssa failed with error code $status")
	end
	return lower[1], upper[1]
end

function getBasisBoundsCplex{DIM}(c::Array{JuMP.ConstraintRef, DIM})
	if length(c[1].m.linconstrDuals) != MathProgBase.numlinconstr(c[1].m)
        error("Sensitivity information not available.")
    end
	numCtr = length(c)
	lowers = Array{Float64}(numCtr)
	uppers = Array{Float64}(numCtr)
	for i = 1:numCtr
		# Gotta call the C API for each ctr and not for all of them at once
		# because the index may not be in order or contiguous
		lowers[i], uppers[i] = getBasisBoundsCplex(c[i])
	end
	return lowers, uppers
end

function getBasisBoundsCplex{DIM, ANY}(c::JuMP.JuMPArray{JuMP.ConstraintRef, DIM, ANY})
	return getBasisBoundsCplex(c.innerArray)
end
