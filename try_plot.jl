#using Plots
#plotly() # Choose the Plotly.jl backend for web interactivity
#plot(rand(5,5),linewidth=2,title="My Plot")

#Pkg.add("PyPlot") # Install a different backend
#pyplot() # Switch to using the PyPlot.jl backend
#plot(rand(5,5),linewidth=2,title="My Plot") # The same plotting command works

#using PyPlot
#x = linspace(0,2*pi,1000); y = sin.(3*x + 4*cos.(2*x))
#plot(x, y, color="red", linewidth=2.0, linestyle="--")

function essaie(a,b)
    c=a+b
    d=a-b
    m=[2*a 2*b]
    return c,d,m
end


essaie(1,2)
