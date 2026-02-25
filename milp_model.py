import gurobipy as gp
from typing import Optional, Dict, List
from collections import defaultdict
from openpyxl import load_workbook
import numpy as np
import math

Node = str
Veh = str
Route = str

class E2EVRP:
    def __init__(self):
        # Name
        self.instance_name: Optional[str] = None

        # Nodes
        self.N: List[Node] = list()
        self.Nd: Optional[Node] = None 
        self.NS: List[Node] = list()
        self.NC: List[Node] = list()
        self.NR: List[Node] = list()

        # Arc parameters
        self.d: Dict[tuple[Node, Node], float] = dict()
        self.dr: Dict[tuple[Node, Node, Node], float] = dict()
        self.t: Dict[tuple[Node, Node], float] = dict()

        # Customer parameters
        self.q: Dict[Node, float] = dict()
        self.S: Dict[Node, float] = dict()
        self.e: Dict[Node, float] = dict()
        self.l: Dict[Node, float] = dict()
        self.p: Dict[Node, float] = dict()

        # Vehicles
        self.V: List[Veh] = list()
        self.Vd: List[Veh] = list()
        self.Vs: Dict[Node, List[Veh]] = dict()

        # Vehicle parameters
        self.Q: Dict[Veh, float] = dict()
        self.L: Dict[Veh, float] = dict()
        self.h: Dict[Veh, float] = dict()
        self.c: Dict[Veh, float] = dict()

        # Constant parameters
        self.eta: Optional[float] = None
        self.rho: Optional[float] = None
        self.T0: Optional[float] = None

    def read_instance(self, name):
        section = None
        coord = {}
        v_count = 0

        instance = f"./e-2e-vrp instances/{name}.txt"
        self.instance_name = name

        with open(instance, "r") as f:
            for raw in f:
                line = raw.strip()

                if not line or line.startswith("!--"):
                    continue

                if line.startswith("!Stores"):
                    section = "stores"
                    continue
                if line.startswith("!Recharge stations"):
                    section = "recharge"
                    continue
                if line.startswith("!Trucks"):
                    section = "trucks"
                    continue
                if line.startswith("!CityFreighters"):
                    section = "cityf"
                    continue
                if line.startswith("!Customers"):
                    section = "cust"
                    continue

                if line.startswith("r "):
                    key, val = line.split()[0], line.split("/")[1]
                    self.rho = float(val)
                    continue
                if line.startswith("g "):
                    key, val = line.split()[0], line.split("/")[1]
                    self.eta = float(val)
                    continue
                if line.startswith("T "):
                    key, val = line.split()[0], line.split("/")[1]
                    self.T0 = float(val)
                    continue
                if line.startswith("v "):
                    key, val = line.split()[0], line.split("/")[1]
                    speed = float(val)
                    continue

                if section == "stores":
                    items = line.split()
                    for idx, item in enumerate(items):
                        x, y = item.split(",")
                        if idx == 0: # Deposit
                            self.Nd = "D0"
                            coord[self.Nd] = (float(x), float(y))
                            self.N.append(self.Nd)
                        else: #Satellites
                            s_id = f"S{idx-1}"
                            coord[s_id] = (float(x), float(y))
                            self.N.append(s_id)
                            self.NS.append(s_id)
                elif section == "recharge":
                    items = line.split()
                    for idx, item in enumerate(items):
                        x, y = item.split(",")
                        r_id = f"R{idx}"
                        coord[r_id] = (float(x), float(y))
                        self.N.append(r_id)
                        self.NR.append(r_id)
                elif section == "trucks":
                    items = line.split()
                    vt = []
                    for idx, item in enumerate(items):
                        tn, tq, th, tc = item.split(",")
                        self.m[self.Nd, f"V{idx}"] = int(tn)
                        self.Qv[self.Nd, f"V{idx}"] = float(tq)
                        self.hv[self.Nd, f"V{idx}"] = float(th)
                        self.cv[self.Nd, f"V{idx}"] = float(tc)
                        vt.append(f"V{idx}")
                        for i in range(int(tn)):
                            v_id = f"R{i+v_count}"
                            self.Q[v_id] = float(tq)
                            self.h[v_id] = float(th)
                            self.c[v_id] = float(tc)
                            self.V.append(v_id)
                            self.Vd.append(v_id)
                        v_count += int(tn)
                    self.Vt[self.Nd] = vt
                elif section == "cityf":
                    items = line.split()
                    idx = 0
                    for s in self.NS:
                        fv = int(items[idx])
                        idx += 1
                        vs = []
                        vt = []
                        for i in range(fv):
                            fn, fq, fb, fh, fc = items[idx].split(",")
                            self.m[s, f"V{i}"] = int(fn)
                            self.Qv[s, f"V{i}"] = float(fq)
                            self.Lv[s, f"V{i}"] = float(fb)
                            self.hv[s, f"V{i}"] = float(fh)
                            self.cv[s, f"V{i}"] = float(fc)
                            vt.append(f"V{i}")
                            for j in range(int(fn)):
                                v_id = f"V{v_count}"
                                self.Q[v_id] = float(fq)
                                self.L[v_id] = float(fb)
                                self.h[v_id] = float(fh)
                                self.c[v_id] = float(fc)
                                self.V.append(v_id)
                                vs.append(v_id)
                                v_count += 1
                            idx += 1
                        self.Vs[s] = vs
                        self.Vt[s] = vt
                elif section == "cust":
                    items = line.split()
                    for idx, item in enumerate(items):
                        x, y, d, rt, dd, st, pc = item.split(",")
                        c_id = f"C{idx}"
                        coord[c_id] = (float(x), float(y))
                        self.q[c_id] = float(d)
                        self.e[c_id] = float(rt)
                        self.l[c_id] = float(dd)
                        self.S[c_id] = float(st)
                        self.p[c_id] = float(pc)
                        self.N.append(c_id)
                        self.NC.append(c_id)
            
            for i in self.N:
                for j in self.N:
                    self.d[i,j] = round(math.sqrt((coord[i][0] - coord[j][0])**2 + (coord[i][1] - coord[j][1])**2), 4)
                    self.t[i,j] = self.d[i,j]/speed

            for i in self.NS+self.NC:
                for j in self.NS+self.NC:
                    for r in ['0']+self.NR:
                        self.dr[i,j,r] = self.d[i,j] if r == '0' else round(math.sqrt((coord[i][0] - coord[r][0])**2 + (coord[i][1] - coord[r][1])**2) +
                                                                            math.sqrt((coord[r][0] - coord[j][0])**2 + (coord[r][1] - coord[j][1])**2), 4)

    def milp_model(self, env_params, time_limit, output=0):
        M = 3*max(self.l.values())
        Mq = max(self.Q.values())
        Ml = max(self.L.values())
          
        env = gp.Env(params=env_params)

        #Model
        model = gp.Model(self.instance_name, env=env)
        model.Params.OutputFlag = output

        #Variables
        w = {(i,s): model.addVar(vtype = gp.GRB.BINARY, name = f"w^{s}({i})") for s in self.NS for i in self.NC}
        a = {s: model.addVar(vtype = gp.GRB.BINARY, name = f"a({s})") for s in self.NS}
        x0 = {(i,j,v): model.addVar(vtype = gp.GRB.BINARY, name = f"x^0({i},{j},{v})") for i in [self.Nd]+self.NS for j in [self.Nd]+self.NS for v in self.Vd if i!=j}
        y0 = {v: model.addVar(vtype = gp.GRB.BINARY, name = f"y^0({v})") for v in self.Vd}
        U0 = {(i,v): model.addVar(vtype = gp.GRB.CONTINUOUS, lb = 0, name = f"U({i},{v})") for i in [self.Nd]+self.NS for v in self.Vd}
        x = {(i,j,r,v,s): model.addVar(vtype = gp.GRB.BINARY, name = f"x^{s}({i},{j},{r},{v})") for s in self.NS for i in [s]+self.NC for j in [s]+self.NC for r in ['0']+self.NR for v in self.Vs[s] if i!=j}
        y = {(v,s): model.addVar(vtype = gp.GRB.BINARY, name = f"y^{s}({v})") for s in self.NS for v in self.Vs[s]}
        U = {(i,v,s): model.addVar(vtype = gp.GRB.CONTINUOUS, lb = 0, name = f"U^{s}({i},{v})") for s in self.NS for i in [s]+self.NC for v in self.Vs[s]}
        E = {(i,v,s): model.addVar(vtype = gp.GRB.CONTINUOUS, lb = 0, name = f"E^{s}({i},{v})") for s in self.NS for i in [s]+self.NC for v in self.Vs[s]}
        T = {i: model.addVar(vtype = gp.GRB.CONTINUOUS, lb = 0, name = f"T({i})") for i in self.NS+self.NC}
        o = {i: model.addVar(vtype = gp.GRB.CONTINUOUS, lb = 0, name = f"T({i})") for i in self.NC}

        #Objective function
        model.setObjective(gp.quicksum(self.h[v]*y0[v] for v in self.Vd) + gp.quicksum(self.h[v]*y[v,s] for s in self.NS for v in self.Vs[s])
                        + gp.quicksum(self.d[i,j]*self.c[v]*x0[i,j,v] for i in [self.Nd]+self.NS for j in [self.Nd]+self.NS for v in self.Vd if i!=j)
                        + gp.quicksum(self.dr[i,j,r]*self.c[v]*x[i,j,r,v,s] for s in self.NS for i in [s]+self.NC for j in [s]+self.NC for r in ['0']+self.NR for v in self.Vs[s] if i!=j)
                        + gp.quicksum(self.p[i]*o[i] for i in self.NC), gp.GRB.MINIMIZE)

        #Constraints
        #Assignament
        for i in self.NC:
            model.addConstr(gp.quicksum(w[i,s] for s in self.NS) == 1) #Satellite - Customer

        for s in self.NS:
            model.addConstr(gp.quicksum(self.q[i]*w[i,s] for i in self.NC) <= Mq*a[s]) #Capacity

        #First echelon
        for s in self.NS:
            model.addConstr(gp.quicksum(x0[i,s,v] for i in [self.Nd]+self.NS if i!=s for v in self.Vd) == a[s]) #Arrives at satellite
            model.addConstr(gp.quicksum(x0[s,i,v] for i in [self.Nd]+self.NS if i!=s for v in self.Vd) == a[s]) #Departs from satellite

            for v in self.Vd:
                model.addConstr(gp.quicksum(x0[i,s,v] for i in [self.Nd]+self.NS if i!=s) - gp.quicksum(x0[s,j,v] for j in [self.Nd]+self.NS if j!=s) == 0) #Flow conservation (satellites)

        for v in self.Vd:
            model.addConstr(gp.quicksum(x0[self.Nd,s,v] for s in self.NS) == y0[v]) #Start at deposit
            model.addConstr(gp.quicksum(x0[s,self.Nd,v] for s in self.NS) == y0[v]) #End at deposit

            for i in [self.Nd]+self.NS:
                model.addConstr(U0[i,v] <= self.Q[v]*y0[v]) #Capacity per route

                for j in self.NS:
                    if i!=j:
                        model.addConstr(U0[j,v] >= U0[i,v] + gp.quicksum(self.q[c]*w[c,j] for c in self.NC) - Mq*(1-x0[i,j,v])) #Transported demand

        #Second-echelon
        for s in self.NS:
            model.addConstr(T[s] >= self.T0) #Minimum hour

            for i in self.NC:
                model.addConstr(gp.quicksum(x[j,i,r,v,s] for j in [s]+self.NC if i!=j for r in ['0']+self.NR for v in self.Vs[s]) == w[i,s]) #Arrives at customer from the satellite
                model.addConstr(gp.quicksum(x[i,j,r,v,s] for j in [s]+self.NC if i!=j for r in ['0']+self.NR for v in self.Vs[s]) == w[i,s]) #Departs from customer to the satellite
                
                for v in self.Vs[s]:
                    model.addConstr(gp.quicksum(x[k,i,r,v,s] for k in [s]+self.NC if i!=k for r in ['0']+self.NR) - gp.quicksum(x[i,j,r,v,s] for j in [s]+self.NC if i!=j for r in ['0']+self.NR) == 0) #Flow conservation (satellites)   
                    
            for v in self.Vs[s]:
                model.addConstr(gp.quicksum(x[s,i,r,v,s] for i in self.NC for r in ['0']+self.NR) == y[v,s]) #Start at satellite
                model.addConstr(gp.quicksum(x[i,s,r,v,s] for i in self.NC for r in ['0']+self.NR) == y[v,s]) #End at satellite

                for i in [s]+self.NC:
                    model.addConstr(U[i,v,s] <= self.Q[v]*y[v,s]) #Capacity per route
                    model.addConstr(E[i,v,s] <= self.L[v]*y[v,s]) #Battery per route

                    for j in self.NC:
                        if i!=j:
                            model.addConstr(E[j,v,s] <= E[i,v,s] - self.rho*self.d[i,j] + Ml*(1-x[i,j,'0',v,s])) #Current battery
                                                        
                            for r in self.NR:
                                model.addConstr(E[j,v,s] <= self.L[v] - self.rho*self.d[r,j] + Ml*(1-x[i,j,r,v,s])) #Current battery
                                model.addConstr(E[i,v,s] >= self.rho*self.d[i,r]*x[i,j,r,v,s]) #Enough battery to arrive

                            for r in ['0']+self.NR:
                                model.addConstr(U[j,v,s] >= U[i,v,s] + self.q[j] - Mq*(1-x[i,j,r,v,s])) #Transported demand

                for i in self.NC:
                    model.addConstr(E[i,v,s] >= self.rho*self.d[i,s]*x[i,s,'0',v,s]) #Enough battery to return
                    model.addConstr(T[i] >= T[s] + self.t[s,i] - M*(1-x[s,i,'0',v,s])) #Cumulative hours

                    for r in self.NR:
                        model.addConstr(self.L[v] >= self.rho*self.d[r,s]*x[i,s,r,v,s]) #Enough battery to return
                        model.addConstr(E[i,v,s] >= self.rho*self.d[i,r]*x[i,s,r,v,s]) #Enough battery to return

                    for j in self.NC:
                        if i!=j:
                            model.addConstr(T[j] >= T[i] + self.S[i] + self.t[i,j] - M*(1-x[i,j,'0',v,s])) #Cumulative hours 

                            for r in self.NR:
                                model.addConstr(T[j] >= T[i] + self.S[i] + self.t[i,r] + self.t[r,j] + self.eta*(self.L[v]-(E[i,v,s]-self.rho*self.d[i,r])) - M*(1-x[i,j,r,v,s])) #Cumulative hours 

        for i in self.NC:
            model.addConstr(T[i] >= self.e[i])
            model.addConstr(T[i] <= self.l[i]+o[i])

        # Addittional constrains
        for v in self.Vd:
            model.addConstr(U0[self.Nd,v] == 0)

        for s in self.NS:
            for v in self.Vs[s]:
                model.addConstr(U[s,v,s] == 0)
                model.addConstr(gp.quicksum(x[s,i,r,v,s] for i in self.NC for r in self.NR) == 0)
                        
        model.update()
        model.Params.TimeLimit = time_limit

        model.optimize()

        if model.Status == gp.GRB.INFEASIBLE:
            self.w = None
            self.a = None
            self.x0 = None
            self.y0 = None
            self.x = None
            self.y = None

            objf = float("inf")
            gap = float("inf")
            exe_time = model.Runtime
        else:
            self.w = {(i,s): w[i,s].X for s in self.NS for i in self.NC}
            self.a = {s: a[s].X for s in self.NS}
            self.x0 = {(i,j,v): x0[i,j,v].X for i in [self.Nd]+self.NS for j in [self.Nd]+self.NS for v in self.Vd if i!=j}
            self.y0 = {v: y0[v].X for v in self.Vd}
            self.x = {(i,j,r,v,s): x[i,j,r,v,s].X for s in self.NS for i in [s]+self.NC for j in [s]+self.NC for r in ['0']+self.NR for v in self.Vs[s] if i!=j}
            self.y = {(v,s): y[v,s].X for s in self.NS for v in self.Vs[s]}

            objf = model.ObjVal
            gap = model.MIPGap
            exe_time = model.Runtime

        return objf, gap, exe_time
    
params = {
    "WLSACCESSID": '84a79f6b-88c8-4dc6-a15d-043da3f6e5f2',
    "WLSSECRET": '59749acc-3ea2-45b9-8322-3f3ecdd7c204',
    "LICENSEID": 939786
}

workbook = load_workbook("./Instances.xlsx")
sheet = workbook['Hoja1']

for i in range(598):
    instance = sheet.cell(2+i, 1).value
    e2evrp = E2EVRP()
    e2evrp.read_instance(instance)
    objf, gap, exe_time = e2evrp.milp_model(params, 3600, 0)

    sheet.cell(2+i, 6, value=objf)
    sheet.cell(2+i, 7, value=gap)
    sheet.cell(2+i, 8, value=exe_time)
    workbook.save("./Instances.xlsx")
