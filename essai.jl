using JuMP
using CPLEX

include("sensitivity_cplex.jl")
m=Model(solver=CplexSolver())
@variable(m,x1>=0)
@variable(m,x2>=0)
@variable(m,x3>=0)
@objective(m,Max,2x1+5x2+1x3)
@constraint(m,A,x1+x3>=1)
@constraint(m,B,-x2+2x3>=0)
@constraint(m,C,3x1+x3<=5)
solve(m)
println(getvalue(x1))
println(getvalue(x2))
println(getvalue(x3))

println(getBasisBoundsCplex(A))
println(getBasisBoundsCplex(B))
println(getBasisBoundsCplex(C))

println(getSlacksCplex(A))
println(getSlacksCplex(B))
println(getSlacksCplex(C))
