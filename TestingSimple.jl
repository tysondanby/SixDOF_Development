# Guide
# Pkg.dev("SixDOF")
# Pkg.add("SixDOF")
# Pkg.test("SixDOF")
# include("SixDOF.jl")

using Plots
```

First, we import the module.

```
using SixDOF
```

There are several structs we need to define.  The inputs used in this example correspond to the Zagi flying wing in Appendix E of Small Unmanned Aircraft: Theory and Practice by Beard and McLain.  We specify the mass properties:

```
```

For this example:
```
m = 1.56
Ixx = 0.1147
Iyy = 0.0576
Izz = 0.1712
Ixz = 0.0015
mp = MassProp(m, Ixx, Iyy, Izz, Ixz)
nothing # hide
```

Next, we specify reference areas and lengths:


```
Sref = 0.2589
bref = 1.4224
cref = 0.3302
ref = Reference(Sref, bref, cref)
nothing # hide

controller = ConstantController(0.0, 0.0, 0.0, 0.0, 0.8)
nothing # hide

Wi = [0.0, 0.0, 0.0]
Wb = [0.0, 0.0, 0.0]
rho = 1.2682
asound = 300.0
g = 9.81
atm = ConstantAtmosphere(Wi, Wb, rho, asound, g)
nothing # hide



CL0 = 0.09167 # Zero-alpha lift
CLalpha = 3.5016  # lift curve slope
CLq = 2.8932 # Pitch rate derivative
CLM = 0.0 # Mach derivative
CLdf = 0.0  # flaps derivative
CLde = 0.2724  # elevator derivative
CLmax = 1.4  # max CL (stall)
CLmin = -0.9  # min CL (neg stall)
alphas = 20*pi/180

CD0 = 0.01631  # zero-lift drag coerff
U0 = 10.0  # velocity corresponding to Reynolds number of CD0
exp_Re = -0.2  # exponent in Reynolds number calc
e = 0.8  # Oswald efficiency
Mcc = 0.7  # crest critcal Mach number
CDdf = 0.0  # flaps
CDde = 0.3045  # elevator
CDda = 0.0  # aileron
CDdr = 0.0  # rudder

CYbeta = -0.07359 # Sideslip derivative
CYp = 0.0  # Roll rate derivative
CYr = 0.0 # Yaw rate derivative
CYda = 0.0 # Roll control (aileron) derivative
CYdr = 0.0 # Yaw control (rudder) derivative

Clbeta = -0.02854  # Sideslip derivative
Clp = -0.3209  # Roll rate derivative
Clr = 0.03066  # Yaw rate derivative
Clda = 0.1682  # Roll (aileron) control derivative
Cldr = 0.0  #Yaw (rudder) control derivative

Cm0 = -0.02338 # Zero-alpha pitch
Cmalpha = -0.5675 # Alpha derivative
Cmq = -1.3990 # Pitch rate derivative
CmM = 0.0
Cmdf = 0.0
Cmde = -0.3254 # Pitch control derivative

Cnbeta = -0.00040  # Slideslip derivative
Cnp = -0.01297  # Roll rate derivative
Cnr = -0.00434  # Yaw rate derivative
Cnda = -0.00328  # Roll (aileron) control derivative
Cndr = 0.0  # Yaw (rudder) control derivative

sd = StabilityDeriv(CL0, CLalpha, CLq, CLM, CLdf, CLde, alphas,
    CD0, U0, exp_Re, e, Mcc, CDdf, CDde, CDda, CDdr,
    CYbeta, CYp, CYr, CYda, CYdr,
    Clbeta, Clp, Clr, Clda, Cldr,
    Cm0, Cmalpha, Cmq, CmM, Cmdf, Cmde,
    Cnbeta, Cnp, Cnr, Cnda, Cndr)
nothing # hide

CT0 = 0.11221
CT1 = -0.13803
CT2 = -0.047394
CQ0 = 0.0062
CQ1 = 0.00314
CQ2 = -0.015729
D = 10*0.0254
num = 2
type = COCOUNTER
R = 0.5
Kv = 2500.0 * pi/30
i0 = 0.3
voltage = 8.0
propulsion = MotorPropBatteryDataFit(CT2, CT1, CT0, CQ2, CQ1, CQ0, D, num, type, R, Kv, i0, voltage)
nothing # hide

inertial2 = UniformGravitationalField()
nothing # hide


Vinf = U0
alpha = 3.0*pi/180
s0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Vinf*cos(alpha), 0.0, Vinf*sin(alpha), 0.0, 0.0, 0.0]
tspan = (0.0, 4.0)
p = mp, ref, sd, propulsion, inertial2, atm, controller

# using DifferentialEquations
# prob = DifferentialEquations.ODEProblem(sixdof!, s0, tspan, p)
# sol = DifferentialEquations.solve(prob)
# nothing # hide

#Tyson testing his stuff
JACOBIAN=GetJacobian(s0,p)

leftWing = wing([.3 0 0],[-.1 -1 .05], .15, .1, -0.5, 3.0)
rightWing =wing([.3 0 0],[-.1  1 .05], .15, .1, -0.5, 3.0)

planeModel = (leftWing,rightWing)

animVTK(JACOBIAN, planeModel)

# using PyPlot
# ```
# figure()
# plot(sol.t, sol[1, :])
# xlabel("time (s)")
# ylabel("x inertial position (m)")
# savefig("x.svg"); nothing # hide
# figure()
# plot(sol.t, sol[3, :])
# xlabel("time (s)")
# ylabel("z inertial position (m)")
# savefig("z.svg"); nothing # hide
# figure()
# plot(sol.t, sol[7, :])
# xlabel("time (s)")
# ylabel("u body velocity (m/s)")
# savefig("u.svg"); nothing # hide
# figure()
# figure()
# plot(sol.t, sol[9, :])
# xlabel("time (s)")
# ylabel("w body velocity (m/s)")
# savefig("w.svg"); nothing # hide
# ```
