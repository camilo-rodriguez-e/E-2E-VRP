#Sets
set N; #Set of Nodes
set NS within N; #Subset of Satellites
set NC within N; #Subset of Customers
set NR within N; #Subset of Recharging stations

set V; #Set of Vehicles
set Vd within V; #Subset of Vehicles from depot
set Vs {s in NS} within V; #Subset of Vehicles based at satellite s

#Parameters
param Nd symbolic in N; #Depot node 

param d{i in N,j in N}; #Distance from i to j
param t{i in N,j in N}; #Time from i to j

param K{s in NS}; #Capacity of satellite s

param q{i in NC}; #Demand of customer c
param e{i in NC}; #Start due date of customer c
param l{i in NC}; #End due date of customer c
param st{i in NC}; #Service time of customer c
param p{i in NC}; #Penalization cost of customer c

param eta; #Recharge rate

param dp{i in NS union NC,j in NS union NC,r in {'0'} union NR}:= if r == '0' then d[i,j] else d[i,r] + d[r,j]; #Distance from i to j recharging in r

param h{v in V}; #Fixed cost of vehicle v
param c{v in V}; #Variable cost of vehicle v
param Q{v in V}; #Capacity of vehicle v
param L{v in V}; #Battery of vehicle v

param T0; #Minimum start time
param rho; # Consume rate per distance

param M:= 3*sum{i in NC} l[i]; #Big-M

#Decision variables
var x0{i in {Nd} union NS,j in {Nd} union NS,v in Vd} binary; #Arc i to j is taken by vehicle v
var y0{v in Vd} binary; #Vehicle v is taken
var U0{i in {Nd} union NS,v in Vd} >= 0; #Cumulative capacity at node i in vehicle v

var w{s in NS, i in NC} binary; #Customer c assigned to satellite s
var a{s in NS} binary; #Satellite s is visited

var x{s in NS,i in {s} union NC,j in {s} union NC,r in {'0'} union NR,v in Vs[s]} binary; #Arc i to j is taken by vehicle v from satellite s
var y{s in NS,v in Vs[s]} binary; #Vehicle v from satellite s is taken
var U{s in NS,i in {s} union NC,v in Vs[s]} >= 0; #Cumulative capacity at node i in vehicle v from satellite s
var E{s in NS,i in {s} union NC,v in Vs[s]} >= 0; #Remaining battery at node i in vehicle v from satellite s

var T{i in NS union NC} >= 0; #Arrival time at node i
var o{i in NC} >= 0; #Tardiness at customer c

#Objective function
minimize Z:
	sum{v in Vd}h[v]*y0[v] + sum{v in Vd}sum{i in {Nd} union NS}sum{j in {Nd} union NS: i<>j}d[i,j]*c[v]*x0[i,j,v] + sum{s in NS}sum{v in Vs[s]}h[v]*y[s,v] + sum{s in NS}sum{v in Vs[s]}sum{i in {s} union NC}sum{j in {s} union NC: i<>j}sum{r in {'0'} union NR}dp[i,j,r]*c[v]*x[s,i,j,r,v] + sum{i in NC}p[i]*o[i];

#Constraints
subject to Assignament{i in NC}:
	sum{s in NS}w[s,i] = 1;

subject to SatelliteCapacity{s in NS}:
	sum{i in NC}q[i]*w[s,i] <= K[s]*a[s];

subject to DepartSatellite{s in NS}:
	sum{v in Vd}sum{i in {Nd} union NS: i<>s}x0[s,i,v] = a[s];

subject to ArriveSatellite{s in NS}:
	sum{v in Vd}sum{i in {Nd} union NS: i<>s}x0[i,s,v] = a[s];

subject to FlowConservation{v in Vd,s in NS}:
	sum{i in {Nd} union NS: i<>s}x0[i,s,v] - sum{j in {Nd} union NS: j<>s}x0[s,j,v] = 0;

subject to DepartDepot{v in Vd}:
	sum{s in NS}x0[Nd,s,v] = y0[v];

subject to ArriveDepot{v in Vd}:
	sum{s in NS}x0[s,Nd,v] = y0[v];

subject to CumulativeSatellite{v in Vd,i in {Nd} union NS,s in NS: i<>s}:
	U0[s,v] >= U0[i,v] + sum{j in NC}q[j]*w[s,j] - Q[v]*(1-x0[i,s,v]);

subject to CumulativeLimit{v in Vd,i in {Nd} union NS}:
	U0[i,v] <= Q[v]*y0[v];

subject to DepartCustomer{s in NS,i in NC}:
	sum{v in Vs[s]}sum{j in {s} union NC: i<>j}sum{r in {'0'} union NR}x[s,i,j,r,v] = w[s,i];

subject to ArriveCustomer{s in NS,i in NC}:
	sum{v in Vs[s]}sum{j in {s} union NC: i<>j}sum{r in {'0'} union NR}x[s,j,i,r,v] = w[s,i];

subject to FlowConservation2{s in NS,v in Vs[s],i in NC}:
	sum{j in {s} union NC: i<>j}sum{r in {'0'} union NR}x[s,j,i,r,v] - sum{k in {s} union NC: i<>k}sum{r in {'0'} union NR}x[s,i,k,r,v] = 0;

subject to DepartSatellite2{s in NS,v in Vs[s]}:
	sum{i in NC}sum{r in {'0'} union NR}x[s,s,i,r,v] = y[s,v];

subject to ArriveSatellite2{s in NS,v in Vs[s]}:
	sum{i in NC}sum{r in {'0'} union NR}x[s,i,s,r,v] = y[s,v];

subject to CumulativeCustomer{s in NS,v in Vs[s],i in {s} union NC,j in NC,r in {'0'} union NR: i<>j}:
	U[s,j,v] >= U[s,i,v] + q[j] - Q[v]*(1-x[s,i,j,r,v]);

subject to CumulativeLimit2{s in NS,v in Vs[s],i in {s} union NC}:
	U[s,i,v] <= Q[v]*y[s,v];

subject to RemainingBattery1{s in NS,v in Vs[s],j in {s} union NC,i in NC:i<>j}:
	E[s,j,v] <= E[s,i,v] - rho*d[i,j] + L[v]*(1-x[s,i,j,'0',v]);

subject to RemainingBattery2{s in NS,v in Vs[s],i in {s} union NC,j in NC,r in NR: i<>j}:
	E[s,j,v] <= L[v] - rho*d[r,j] + L[v]*(1-x[s,i,j,r,v]);

subject to RemainingBattery3{s in NS,v in Vs[s],i in {s} union NC,j in NC: i<>j}:
	E[s,i,v] >= rho*d[i,j]*x[s,i,j,'0',v];

subject to RemainingBattery4{s in NS,v in Vs[s],i in {s} union NC,j in NC,r in NR: i<>j}:
	E[s,i,v] >= rho*d[i,r]*x[s,i,j,r,v];

subject to RemainingBattery5{s in NS,v in Vs[s],i in NC}:
	E[s,i,v] >= rho*d[i,s]*x[s,i,s,'0',v];

subject to RemainingBattery6{s in NS,v in Vs[s],i in NC,r in NR}:
	L[v] >= rho*d[r,s]*x[s,i,s,r,v];

subject to BatteryLimit{s in NS,v in Vs[s],i in {s} union NC}:
	E[s,i,v] <= L[v]*y[s,v];

subject to ArrivalTime1{s in NS}:
	T[s] >= T0;

subject to ArrivalTime2{s in NS,v in Vs[s],i in NC,j in NC: i<>j}:
	T[j] >= T[i] + st[i] + t[i,j] - M*(1-x[s,i,j,'0',v]);

subject to ArrivalTime3{s in NS,v in Vs[s],i in NC,j in NC,r in NR: i<>j}:
	T[j] >= T[i] + st[i] + t[i,r] + eta*(L[v] - (E[s,i,v] - rho*d[i,r])) + t[r,j] - M*(1-x[s,i,j,r,v]);

subject to ArrivalTime4{s in NS,v in Vs[s],i in NC}:
	T[i] >= T[s] + t[s,i] - M*(1-x[s,s,i,'0',v]);

subject to ArrivalTime5{s in NS,v in Vs[s],i in NC,r in NR}:
	T[i] >= T[s] + t[s,r] + eta*(L[v] - (E[s,s,v] - rho*d[s,r])) + t[r,i] - M*(1-x[s,s,i,r,v]);

subject to MinimumTime{i in NC}:
	T[i] >= e[i];

subject to MaximumTime{i in NC}:
	T[i] <= l[i] + o[i];

#Addittional constrains
subject to InitialValueDepot{v in Vd}:
	U0[Nd,v] = 0;

subject to InitialValueSatellite{s in NS,v in Vs[s]}:
	U[s,s,v] = 0;

subject to NoRechargeAtBeginning{s in NS,v in Vs[s]}:
	sum{i in NC}sum{r in NR}x[s,s,i,r,v] = 0;
