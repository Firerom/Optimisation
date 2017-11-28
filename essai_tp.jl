Pkg.add("Mosek")

try
 using JLD
catch
 Pkg.add("JLD")
 using JLD
end

try
 using JuMP
catch
 Pkg.add("JuMP")
 using JuMP
end

try
 using Gurobi
catch
 Pkg.add("Gurobi")
 using Gurobi
end

try
 using CPLEX
catch
 Pkg.add("CPLEX")
 using CPLEX
end

try
 using SCS
catch
 Pkg.add("SCS")
 using SCS
end

#m = Model(solver=GurobiSolver())
m = Model(solver=SCSSolver())
@variable(m, M[1:3,1:3], Symmetric)
@SDconstraint(m,M>=0)
@constraint(m,M[1,2]==-1)
@constraint(m,M[1,1]==2)
@constraint(m,M[2,2]+2*M[1,3]==4)
@constraint(m,M[2,3]==-1)
@constraint(m,M[3,3]==2)
solve(m)
