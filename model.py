import matplotlib.pyplot as plt
import gurobipy as gp
import itertools
import math
import os
import random
import time

class E2EVRP:
    def __init__(self, folder, name, params=None):
        # Gurobi env
        self.env = gp.Env(empty=True)
        self.env.setParam("OutputFlag", 0)
        for key, value in (params or {}).items():
            self.env.setParam(key, value)
        self.env.start()

        # Read path
        instance_path = f"./instances/{folder}/{name}.txt"
        if not os.path.exists(instance_path):
            raise FileNotFoundError(f"{instance_path} was not found.")

        self.instance_name = name
        section = None

        # Read parameters

        # Nodes
        self.N = []
        self.Nd = None 
        self.NS = []
        self.NC = []
        self.NR = []
        self.coord = {}

        # Satellite parameters
        self.K = {}

        # Customer parameters
        self.q = {}
        self.S = {}
        self.e = {}
        self.l = {}
        self.p = {}

        # Vehicles
        self.V = []
        self.V1 = []
        self.V2 = []
        
        # Vehicle parameters
        self.Q = {}
        self.h = {}
        self.c = {}
        self.L = {}
        self.r = {}

        # Constant parameters
        self.eta = None

        with open(instance_path, "r", encoding="utf-8") as f:
            for raw in f:
                line = raw.strip()

                if not line or line.startswith("!--"):
                    continue

                if line.startswith("!Stores"):
                    section = "stores"
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
                if line.startswith("!Recharge stations"):
                    section = "recharge"
                    continue

                if line.startswith("g "):
                    parts = line.strip().split()
                    self.eta = float(parts[-1])
                    continue

                if section == "stores":
                    items = line.split()
                    for idx, item in enumerate(items):
                        if idx == 0: # Deposit
                            x, y = item.split(",")
                            self.Nd = "D0"
                            self.coord[self.Nd] = (float(x), float(y))
                            self.N.append(self.Nd)
                        else: # Satellites
                            x, y, maxCap = item.split(",")
                            s_id = f"S{idx-1}"
                            self.coord[s_id] = (float(x), float(y))
                            self.K[s_id] = float(maxCap)
                            self.N.append(s_id)
                            self.NS.append(s_id)
                elif section == "trucks":
                    items = line.split()
                    for idx, item in enumerate(items):
                        tq, tc, th = item.split(",")
                        ttype = f"T{idx}"
                        self.Q[ttype] = float(tq)
                        self.h[ttype] = float(th)
                        self.c[ttype] = float(tc)
                        self.V.append(ttype)
                        self.V1.append(ttype)
                elif section == "cityf":
                    items = line.split()
                    for idx, item in enumerate(items):
                        fq, fc, fh, fb, fr = item.split(",")
                        ftype = f"F{idx}"
                        self.Q[ftype] = float(fq)
                        self.h[ftype] = float(fh)
                        self.c[ftype] = float(fc)
                        self.L[ftype] = float(fb)
                        self.r[ftype] = float(fr)
                        self.V.append(ftype)
                        self.V2.append(ftype)
                elif section == "cust":
                    items = line.split()
                    for idx, item in enumerate(items):
                        x, y, d, rt, dd, st, pc = item.split(",")
                        c_id = f"C{idx}"
                        self.coord[c_id] = (float(x), float(y))
                        self.q[c_id] = float(d)
                        self.e[c_id] = float(rt)
                        self.l[c_id] = float(dd)
                        self.S[c_id] = float(st)
                        self.p[c_id] = float(pc)
                        self.N.append(c_id)
                        self.NC.append(c_id)
                elif section == "recharge":
                    items = line.split()
                    for idx, item in enumerate(items):
                        x, y = item.split(",")
                        r_id = f"R{idx}"
                        self.coord[r_id] = (float(x), float(y))
                        self.N.append(r_id)
                        self.NR.append(r_id)
                else:
                    continue

        # Arc parameters
        self.d = {}
        self.dr = {}
        self.t = {}

        for i in self.N:
            xi, yi = self.coord[i]
            for j in self.N:
                xj, yj = self.coord[j]
                dij = round(math.sqrt((xi-xj)**2 + (yi-yj)**2))
                self.d[i,j] = dij
                self.t[i,j] = dij
        
        for i in self.NS+self.NC:
            xi, yi = self.coord[i]
            for j in self.NS+self.NC:
                self.dr[i,j,'0'] = self.d[i,j]
                for r in self.NR:
                    self.dr[i,j,r] = self.d[i,r] + self.d[r,j]

        # Matheuristic
        self.pq = 0.25
        self.max = 50
        self.op = [(r, i) for r in ['Rnd','Dist-r','W-Dist','TW-r','W-Lat']
                        for i in ['Rnd','Dist','s-TW','e-TW']]
        self.alpha = 0.005
        self.H = {o: 0 for o in self.op}
        self.R = 0
        self.n = 0
        self.deadline = float("inf")
        self.eval = set()

        # Routes
        self.routes = {}
        self.Rd = {}
        self.C = {}
        self.vt= {}

        # Cache
        self.cov = {}
        self.stn = {}
        self.pulse_cache = {}

    def milp_model(self, time_limit):
        Qmax = max(self.Q.values())
        Lmax = max(self.L.values())
        rmax = max(self.r.values())
        qmax= max(self.q.values())
        dmax = max(self.d.values())
        Tmax = sum(self.S[i] + 2*max(self.t[i,j] for j in self.NC if i != j) for i in self.NC)

        M = 10**(math.floor(math.log10(Tmax)) + 2)
        Mq = 10**(math.floor(math.log10(Qmax + qmax)) + 1)
        Ml = 10**(math.floor(math.log10(Lmax + rmax*dmax)) + 1)

        N1 = [self.Nd]+self.NS
        N2 = self.NS+self.NC
        NR_ = ['0']+self.NR

        # Model
        model = gp.Model(self.instance_name, env=self.env)
        model.Params.TimeLimit = time_limit

        # Decision variables
        w = {(i,s): model.addVar(vtype = gp.GRB.BINARY, name = f"w({i},{s})") for i in self.NC for s in self.NS}
        a = {s: model.addVar(vtype = gp.GRB.BINARY, name = f"a({s})") for s in self.NS}
        x1 = {(i,j,v): model.addVar(vtype = gp.GRB.BINARY, name = f"x^D({i},{j},{v})") for i in N1 for j in N1 for v in self.V1 if i!=j}
        y1 = {v: model.addVar(vtype = gp.GRB.INTEGER, name = f"y^D({v})") for v in self.V1}
        U1 = {i: model.addVar(vtype = gp.GRB.CONTINUOUS, lb = 0, name = f"U^1({i})") for i in N1}
        x2 = {(i,j,r,v,s): model.addVar(vtype = gp.GRB.BINARY, name = f"x^{s}({i},{j},{r},{v})") for s in self.NS for i in [s]+self.NC for j in [s]+self.NC if i!=j for r in NR_ for v in self.V2}
        y2 = {(v,s): model.addVar(vtype = gp.GRB.INTEGER, name = f"y^{s}({v})") for s in self.NS for v in self.V2}
        U2 = {i: model.addVar(vtype = gp.GRB.CONTINUOUS, lb = 0, name = f"U^2({i})") for i in N2}
        E = {i: model.addVar(vtype = gp.GRB.CONTINUOUS, lb = 0, name = f"E({i})") for i in N2}
        T = {i: model.addVar(vtype = gp.GRB.CONTINUOUS, lb = 0, name = f"T({i})") for i in N2}
        o = {i: model.addVar(vtype = gp.GRB.CONTINUOUS, lb = 0, name = f"o({i})") for i in self.NC}

        # Objective function
        model.setObjective(gp.quicksum(self.h[v]*y1[v] for v in self.V1) + gp.quicksum(self.h[v]*y2[v,s] for v in self.V2 for s in self.NS) + 
                           gp.quicksum(self.d[i,j]*self.c[v]*x1[i,j,v] for i in N1 for j in N1 for v in self.V1 if i!=j) +
                           gp.quicksum(self.dr[i,j,r]*self.c[v]*x2[i,j,r,v,s] for s in self.NS for i in [s]+self.NC for j in [s]+self.NC if i!=j for r in NR_ for v in self.V2) +
                           gp.quicksum(self.p[i]*o[i] for i in self.NC), gp.GRB.MINIMIZE)

        # Constraints

        # Assignament
        # Assign satellite with customer
        for i in self.NC:
            model.addConstr(gp.quicksum(w[i,s] for s in self.NS) == 1)

        # Visit satellite if a customer is assigned
        for s in self.NS:
            model.addConstr(gp.quicksum(self.q[i]*w[i,s] for i in self.NC) <= self.K[s]*a[s])

        # First echelon
        # Arrives and departures at satellites
        for s in self.NS:
            model.addConstr(gp.quicksum(x1[i,s,v] for i in N1 if i!=s for v in self.V1) == a[s])
            model.addConstr(gp.quicksum(x1[s,i,v] for i in N1 if i!=s for v in self.V1) == a[s])

        # Arrives and departures at deposit per route
        for v in self.V1:
            model.addConstr(gp.quicksum(x1[self.Nd,s,v] for s in self.NS) == y1[v])
            model.addConstr(gp.quicksum(x1[s,self.Nd,v] for s in self.NS) == y1[v])

        # Flow conservation at satellites
        for s in self.NS:
            for v in self.V1:
                model.addConstr(gp.quicksum(x1[i,s,v] for i in N1 if i!=s) - gp.quicksum(x1[s,j,v] for j in N1 if j!=s) == 0)

        # Transported demand
        model.addConstr(U1[self.Nd] == 0)

        for i in N1:
            model.addConstr(U1[i] <= gp.quicksum(self.Q[v]*x1[i,j,v] for j in N1 if i!=j for v in self.V1))

            for j in self.NS:
                if i!=j:
                    for v in self.V1:
                        model.addConstr(U1[j] >= U1[i] + gp.quicksum(self.q[c]*w[c,j] for c in self.NC) - Mq*(1-x1[i,j,v]))

        # Second-echelon

        # Arrives and departures at customers
        for i in self.NC:
            for s in self.NS:
                model.addConstr(gp.quicksum(x2[j,i,r,v,s] for j in [s]+self.NC if i!=j for r in NR_ for v in self.V2) == w[i,s])
                model.addConstr(gp.quicksum(x2[i,j,r,v,s] for j in [s]+self.NC if i!=j for r in NR_ for v in self.V2) == w[i,s])

        # Arrives and departures at satellites per route
        for s in self.NS:
            for v in self.V2:
                model.addConstr(gp.quicksum(x2[s,i,r,v,s] for i in self.NC for r in NR_) == y2[v,s])
                model.addConstr(gp.quicksum(x2[i,s,r,v,s] for i in self.NC for r in NR_) == y2[v,s])

        # Flow conservation at customers
        for i in self.NC:
            for s in self.NS:
                for v in self.V2:
                    model.addConstr(gp.quicksum(x2[j,i,r,v,s] for j in [s]+self.NC if i!=j for r in NR_) - gp.quicksum(x2[i,k,r,v,s] for k in [s]+self.NC if i!=k for r in NR_) == 0)

        # Transported demand
        for s in self.NS:
            model.addConstr(U2[s] == 0)

        for i in self.NC:
            model.addConstr(U2[i] <= gp.quicksum(x2[i,j,r,v,s]*self.Q[v] for s in self.NS for j in [s]+self.NC if i!=j for r in NR_ for v in self.V2))

            for s in self.NS:
                for j in [s]+self.NC:
                    if i!=j:
                        for r in NR_:
                            for v in self.V2:
                                model.addConstr(U2[i] >= U2[j] + self.q[i] - Mq*(1-x2[j,i,r,v,s]))

        # Energy level
        for s in self.NS:
            model.addConstr(E[s] == 0)

        for i in self.NC:
            model.addConstr(E[i] <= gp.quicksum(x2[i,j,r,v,s]*self.L[v] for s in self.NS for j in [s]+self.NC if i!=j for r in NR_ for v in self.V2))

            for s in self.NS:
                for j in [s]+self.NC:
                    if i!=j:
                        for v in self.V2:
                            model.addConstr(E[i] >= E[j] + self.r[v]*self.d[j,i] - Ml*(1-x2[j,i,'0',v,s]))

                            for r in self.NR:
                                model.addConstr(self.L[v] - E[j] >= self.r[v]*self.d[j,r] - Ml*(1-x2[j,i,r,v,s]))
                                model.addConstr(E[i] >= self.r[v]*self.d[r,i] - Ml*(1-x2[j,i,r,v,s]))

                for v in self.V2:
                    model.addConstr(self.L[v] - E[i] >= self.r[v]*self.d[i,s] - Ml*(1-x2[i,s,'0',v,s]))

                    for r in self.NR:
                        model.addConstr(self.L[v] - E[i] >= self.r[v]*self.d[i,r] - Ml*(1-x2[i,s,r,v,s]))
                        model.addConstr(self.L[v] >= self.r[v]*self.d[r,s]*x2[i,s,r,v,s])

        # Arrive hours
        for s in self.NS:
            model.addConstr(T[s] == 0)

        for i in self.NC:
            for s in self.NS:
                for v in self.V2:
                    model.addConstr(T[i] >= T[s] + self.t[s,i] - M*(1-x2[s,i,'0',v,s]))

                    for r in self.NR:
                        model.addConstr(T[i] >= T[s] + self.t[s,r] + self.eta*self.r[v]*self.d[s,r] + self.t[r,i] - M*(1-x2[s,i,r,v,s]))

            for j in self.NC:
                if i!=j:
                    for s in self.NS:
                        for v in self.V2:
                            model.addConstr(T[j] >= T[i] + self.S[i] + self.t[i,j] - M*(1-x2[i,j,'0',v,s]))

                            for r in self.NR:
                                model.addConstr(T[j] >= T[i] + self.S[i] + self.t[i,r] + self.eta*(E[i] + self.r[v]*self.d[i,r]) + self.t[r,j] - M*(1-x2[i,j,r,v,s]))

            model.addConstr(T[i] >= self.e[i])
            model.addConstr(T[i] <= self.l[i] + o[i])
                             
        model.update()
        model.optimize()

        exe_time = model.Runtime

        if model.SolCount > 0:
            objf = model.ObjVal
            gap = model.MIPGap

            x_ = []

            for v in self.V1:
                for i in self.NS:
                    if x1[self.Nd,i,v].X > 0.9:
                        route = [self.Nd,i]
                        s = i

                        while True:
                            next_node = None
    
                            for j in [self.Nd] + self.NS:
                                if j != s:
                                    if x1[s,j,v].X > 0.9:
                                        next_node = j
                                        break

                            route.append(next_node)
                            s = next_node
                    
                            if s == self.Nd:
                                x_.append({"Vehicle type": v, "Route": route})
                                break

            for s in self.NS:
                for v in self.V2:
                    for i in self.NC:
                        for r in NR_:
                            if x2[s,i,r,v,s].X > 0.9:
                                route = [s,i] if r == '0' else [s,r,i]
                                c = i
            
                                while True:
                                    next_node = None
                                    recharge = None
                
                                    for j in [s] + self.NC:
                                        if j != c:
                                            for r2 in NR_:
                                                if x2[c,j,r2,v,s].X > 0.9:
                                                    next_node = j
                                                    recharge = r2 if r2 != '0' else None
                                                    break

                                            if next_node is not None:
                                                break

                                    if recharge is None:
                                        route.append(next_node)
                                    else:
                                        route.append(recharge)
                                        route.append(next_node)

                                    c = next_node
                                
                                    if c == s:
                                        x_.append({"Vehicle type": v, "Route": route})
                                        break

        else:
            objf = float("inf")
            gap = float("inf")

            x_ = None               

        model.dispose()

        return x_, objf, gap, exe_time

    def graph_route(self, routes, path=None):
        fig, ax = plt.subplots(figsize=(12, 12))

        # Nodes on the 2D map
        groups = [([self.Nd], "s", "darkturquoise", "Depot"),
                (self.NS, "D", "yellowgreen", "Satellites"),
                (self.NC, "o", "lightblue", "Customers")]

        for nodes, marker, color, label in groups:
            if nodes:
                xs, ys = zip(*[self.coord[n] for n in nodes])
                ax.scatter(xs, ys, marker=marker, c=color, s=350, edgecolors="black", label=label, zorder=3)
                for n in nodes:
                    ax.annotate(n, self.coord[n], ha="center", va="center", fontsize=8, fontweight="bold", zorder=4)

        # Routes: 1st level starts at the depot (solid), 2nd level starts at a satellite (dotted, one color per satellite)
        sat_colors = {}
        for r in routes:
            route = r["Route"]
            first = route[0] == self.Nd

            if first:
                color = "tab:gray"
            else:
                color = plt.cm.Dark2(self.NS.index(route[0]) % 8)
                sat_colors[route[0]] = color

            # Recharge stops are not drawn as arcs, they are written in the middle of the line
            prev, recharge = route[0], None
            for n in route[1:]:
                if n in self.NR:
                    recharge = n
                    continue

                ax.annotate("", xy=self.coord[n], xytext=self.coord[prev], zorder=2,
                            arrowprops=dict(arrowstyle="-|>", color=color, lw=2,
                                            linestyle="-" if first else ":", shrinkA=10, shrinkB=10))

                if recharge:
                    (x1, y1), (x2, y2) = self.coord[prev], self.coord[n]
                    ax.text((x1+x2)/2, (y1+y2)/2, recharge, ha="center", va="center", fontsize=8, zorder=5,
                            bbox=dict(boxstyle="round,pad=0.2", facecolor="white", edgecolor=color, lw=1))

                prev, recharge = n, None

        # Line style legend
        ax.plot([], [], color="tab:gray", linestyle="-", lw=2, label="1st level routes")
        for s_, color in sorted(sat_colors.items()):
            ax.plot([], [], color=color, linestyle=":", lw=2, label=f"2nd level routes ({s_})")

        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(alpha=0.3)
        ax.legend(loc="center left", bbox_to_anchor=(1.01, 0.5), fontsize=9)
        ax.set_title(f"Instance: {self.instance_name}", fontsize=14)

        if path:
            os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
            fig.savefig(path, dpi=300, bbox_inches="tight")
            plt.close(fig)
        else:
            plt.show()

    def get_route_1st(self, order, ws, q):
        Q = sum(q[s] for s in order)
        vt = [v for v in self.V1 if self.Q[v] >= Q]

        if not vt:
            return None, None, float("inf"), 0

        route = [self.Nd, *order, self.Nd]

        best_veh, best_costs = None, float("inf")
        duals = sum(ws[s] for s in order)

        for v in vt:
            c = self.c[v]
            costs = self.h[v] 

            for i, j in zip(route, route[1:]):
                costs += c*self.d[i,j]

            if costs < best_costs:
                best_veh, best_costs = v, costs
        
        return route, best_veh, best_costs, duals

    def LNS_1st(self, Q, a, ws):
        found_routes = {}

        NS = [s for s in self.NS if a[s] > 0.9]
        if not NS:
            return []

        S_in, S_out = [], []
        for i in NS:
            (S_in if random.random() < 0.5 else S_out).append(i)

        random.shuffle(S_in)
        best_solution = self.get_route_1st(S_in, ws, Q)

        if best_solution[0] is None:
            while S_in:
                s = random.choice(S_in)
                S_in.remove(s)
                S_out.append(s)
                best_solution = self.get_route_1st(S_in, ws, Q)
                if best_solution[0] is not None:
                    break
        
        if best_solution[0] is None:
            return []

        if best_solution[2] - best_solution[3] < -1e-2:
            found_routes[tuple(best_solution[0])] = best_solution

        p = self.pq
        max_ite = self.max
        not_improve = 0

        while not_improve < max_ite and time.perf_counter() < self.deadline:
            # Remove
            if S_in:
                n = min(round(p*len(S_in)), len(S_in)-1)
                S_rem = random.sample(S_in, n)

                for s in S_rem: 
                    S_in.remove(s) 
                    S_out.append(s)
                
            # Insert
            random.shuffle(S_out)

            curr_solution = self.get_route_1st(S_in, ws, Q)
            curr_S = S_in.copy()

            for s_ in S_out:
                for s in range(len(S_in) + 1):
                    S_new = S_in[:s] + [s_] + S_in[s:]
                    new_solution = self.get_route_1st(S_new, ws, Q)

                    if new_solution[2] - new_solution[3] < curr_solution[2] - curr_solution[3]:
                        curr_solution, curr_S = new_solution, S_new.copy()

                S_in = curr_S.copy()

            S_out = [s for s in S_out if s not in S_in]

            # Save route
            if curr_solution[0] is not None and curr_solution[2] - curr_solution[3] < -1e-2:
                found_routes[tuple(curr_solution[0])] = curr_solution

            # Obtain best solution
            if curr_solution[2] - curr_solution[3] < best_solution[2] - best_solution[3]:
                best_solution = curr_solution
                not_improve = 0
            else:
                not_improve += 1

        return [(sol[0], sol[1], sol[2]) for sol in sorted(found_routes.values(), key=lambda sol: sol[2] - sol[3])[:10]]

    def set_partitioning_1st(self, w):
        # Model
        sp_model = gp.Model("Subproblem: Set partitioning (Depot)", env=self.env)
        sp_model.Params.MIPGap = 0

        # Demand per stop
        Q = {s: sum(self.q[i]*w[i,s] for i in self.NC) for s in self.NS}

        a = {s: max(w[i,s] for i in self.NC) for s in self.NS}

        key = tuple(Q[s] for s in self.NS)
        price = key not in self.eval
        self.eval.add(key)

        act = {s for s in self.NS if a[s] > 0.9}

        # Variables
        R_ = []
        x = {}

        for r in self.routes[self.Nd]:
            Rd_ = self.Rd[r,self.Nd]
            q = sum(Q[s] for s in Rd_[1:-1])
            v = self.vt[r,self.Nd]

            if self.Q[v] >= q and set(Rd_[1:-1]) <= act:
                R_.append(r)
                x[r] = sp_model.addVar(vtype = gp.GRB.CONTINUOUS, name = f"x({r})")

        #Objective function
        sp_model.setObjective(gp.quicksum(self.C[r,self.Nd]*x[r] for r in R_), gp.GRB.MINIMIZE)

        #Constraints
        cover = {s: [] for s in self.NS}
        for r in R_:
            for s in self.Rd[r,self.Nd][1:-1]:
                cover[s].append(x[r])

        LocCtr_c = {}

        for s in self.NS:
            LocCtr_c[s] = sp_model.addConstr(gp.quicksum(cover[s]) == a[s])

        Optimal = False if price else True

        while not Optimal and time.perf_counter() < self.deadline:
            sp_model.Params.TimeLimit = max(self.deadline - time.perf_counter(), 0.1)
            sp_model.update()
            sp_model.optimize()

            if sp_model.Status != gp.GRB.OPTIMAL:
                break

            Optimal = True
            
            ws = {s: LocCtr_c[s].Pi for s in self.NS}

            new_cols = self.LNS_1st(Q, a, ws)

            for new_r, veh, cost in new_cols:
                r_id = f"r{len(self.routes[self.Nd])}"
                self.routes[self.Nd].append(r_id)
                R_.append(r_id)
                self.Rd[r_id,self.Nd] = new_r
                self.vt[r_id,self.Nd] = veh
                self.C[r_id,self.Nd] = cost
                Optimal = False

                newCol = gp.Column()
                newCol.addTerms([1.0]*len(new_r[1:-1]), [LocCtr_c[s] for s in new_r[1:-1]])

                x[r_id] = sp_model.addVar(vtype = gp.GRB.CONTINUOUS, obj=cost, name = f"x({r_id})", column = newCol)

        for r in R_:
            x[r].setAttr("vtype", gp.GRB.BINARY)

        sp_model.Params.TimeLimit = max(self.deadline - time.perf_counter(), 0.1)
        sp_model.update()
        sp_model.optimize()

        if sp_model.SolCount == 0:
            sp_model.dispose()
            return None, float("inf")
        
        r_ = []
        for r in R_:
            if x[r].X > 0.9:
                r_.append(r)

        z_ = sp_model.ObjVal
        sp_model.dispose()
                
        return r_, z_

    def rec_stat(self, i, j):
        if (i,j) not in self.stn:
            keep, best = [], float("inf")

            for r in sorted(self.NR, key=lambda r: (self.d[i,r], self.d[r,j])):
                if self.d[r,j] < best:
                    keep.append(r)
                    best = self.d[r,j]

            self.stn[i,j] = keep

        return self.stn[i,j]

    def elecPULSE(self, order, v):
        key = (tuple(order), v)
        if key in self.pulse_cache:
            cost, route = self.pulse_cache[key]
            return cost, list(route) if route else None

        class PULSE:
            def __init__(self, order, rec_stat, d, t, e, l, S, p, c, L, r, eta):
                self.order = order
                self.stations = rec_stat
                self.d = d
                self.t = t
                self.e = e
                self.l = l
                self.S = S
                self.p = p
                self.c = c
                self.L = L
                self.r = r
                self.eta = eta

                self.lb = {n: 0.0 for n in range(len(order))}
                for n in range(len(self.order) - 2, -1, -1):
                    i, j = order[n], order[n + 1]
                    self.lb[n] = self.lb[n + 1] + self.c*self.d[i,j]

                self.labels = {n: [(float("inf"), float("inf"), 0)] for n in range(len(order))}

                self.ub = float("inf")
                self.best_order = None

            def pulse(self, n, cost, time, battery, path):
                if battery < 0:
                    return
                else:
                    if cost + self.lb[n] >= self.ub:
                        return
                    else:
                        for c_, t_, b_ in self.labels[n]:
                            if c_ <= cost and t_ <= time and b_ >= battery:
                                return

                        self.labels[n] = [(c, t, b) for c, t, b in self.labels[n] if not (cost <= c and time <= t and battery >= b)]
                        self.labels[n].append((cost, time, battery))

                        if n == len(order) - 1:
                            if cost < self.ub:
                                self.ub = cost
                                self.best_order = path.copy()
                            return

                        i, j = order[n], order[n + 1]

                        # Direct
                        if n + 1 == len(order) - 1:
                            new_time = float("inf")
                            new_cost = cost + self.c*self.d[i,j]
                            new_battery = battery - self.r*self.d[i,j]

                            self.pulse(n + 1, new_cost, new_time, new_battery, path + [j])
                        else:
                            new_time = max(time + self.t[i,j], self.e[j])
                            new_cost = cost + self.c*self.d[i,j] + self.p[j]*max(new_time - self.l[j], 0)
                            new_time += self.S[j]
                            new_battery = battery - self.r*self.d[i,j]

                            self.pulse(n + 1, new_cost, new_time, new_battery, path + [j])

                        # Recharge stations
                        for r in self.stations(i, j):
                            if battery >= self.r*self.d[i,r]:
                                if n + 1 == len(order) - 1:
                                    new_time = float("inf")
                                    new_cost = cost + self.c*(self.d[i,r] + self.d[r,j])
                                    new_battery = self.L - self.r*self.d[r,j]
                                
                                    self.pulse(n + 1, new_cost, new_time, new_battery, path + [r,j])
                                else:
                                    new_time = max(time + self.t[i,r] + self.eta*(self.L - (battery - self.r*self.d[i,r])) + self.t[r,j], self.e[j])
                                    new_cost = cost + self.c*(self.d[i,r] + self.d[r,j]) + self.p[j]*max(new_time - self.l[j], 0)
                                    new_time += self.S[j]
                                    new_battery = self.L - self.r*self.d[r,j]
                                
                                    self.pulse(n + 1, new_cost, new_time, new_battery, path + [r,j])

            def solve(self):
                self.pulse(0, 0, 0, self.L, [self.order[0]])
                return self.ub, self.best_order

        ePULSE = PULSE(order, self.rec_stat, self.d, self.t, self.e, self.l, self.S, self.p, self.c[v], self.L[v], self.r[v], self.eta)
        cost, route = ePULSE.solve()
        cost += self.h[v]

        self.pulse_cache[key] = (cost, list(route) if route else None)

        return cost, route

    def get_route_2nd(self, s, order, wc):
        Q = sum(self.q[i] for i in order)    
        vt = [v for v in self.V2 if self.Q[v] >= Q]

        if not vt:
            return None, None, float("inf"), 0

        base = [s,*order,s]

        best_route, best_veh = None, None
        best_costs, duals = float("inf"), sum(wc[i] for i in order)

        for v in vt:
            costs, route = self.elecPULSE(base, v)

            if costs < best_costs:
                best_route, best_veh = route, v
                best_costs = costs

        if best_route is None:
            return None, None, float("inf"), 0

        return best_route, best_veh, best_costs, duals

    def ALNS_2nd(self, s, w, wc):
        found_routes = {}

        NC = [i for i in self.NC if w[i,s] > 0.9]
        if not NC:
            return []

        C_in, C_out = [], []
        for i in NC:
            (C_in if random.random() < 0.5 else C_out).append(i)

        random.shuffle(C_in)

        best_solution = self.get_route_2nd(s, C_in, wc)

        if best_solution[0] is None:
            while C_in:
                c = random.choice(C_in)
                C_in.remove(c)
                C_out.append(c)
                best_solution = self.get_route_2nd(s, C_in, wc)
                if best_solution[0] is not None:
                    break
        
        if best_solution[0] is None:
            return []

        if best_solution[2] - best_solution[3] < -1e-2:
            found_routes[tuple(best_solution[0])] = best_solution

        p = self.pq
        max_ite = self.max
        not_improve = 0

        while not_improve < max_ite and time.perf_counter() < self.deadline:
            Hmax = max(self.H[o] for o in self.op)
            pi = {o: math.exp(self.H[o]-Hmax)/sum(math.exp(self.H[o_]-Hmax) for o_ in self.op) for o in self.op}
            rem, ins = random.choices(list(pi.keys()), weights=list(pi.values()))[0]

            # Remove
            if C_in:
                n = min(round(p*len(C_in)), len(C_in)-1)

                if rem == 'Rnd':
                    C_rem = random.sample(C_in, n)
                elif rem == 'Dist-r':
                    seed = random.choice(C_in)
                    C_rem = sorted(C_in, key=lambda i: self.d[seed,i])[:n]
                elif rem == 'W-Dist':
                    dist = {c: self.d[s,c] if i == 0 else self.d[C_in[i-1],c] for i, c in enumerate(C_in)}
                    C_rem = sorted(C_in, key=lambda i: dist[i], reverse=True)[:n]
                elif rem == 'TW-r':
                    seed = random.choice(C_in)
                    C_rem = sorted(C_in, key=lambda i: abs(self.l[seed]-self.l[i]))[:n]
                elif rem == 'W-Lat':
                    lat, t = {}, 0
                    for i in range(len(C_in)):
                        t = max(self.e[C_in[i]], t + (self.t[s,C_in[i]] if i == 0 else self.S[C_in[i-1]] + self.t[C_in[i-1],C_in[i]]))
                        lat[C_in[i]] = max(t - self.l[C_in[i]], 0)
                    C_rem = sorted(C_in, key=lambda i: lat[i], reverse=True)[:n]

                for c in C_rem: 
                    C_in.remove(c) 
                    C_out.append(c)
                
            # Insert
            if ins == 'Rnd':
                random.shuffle(C_out)
            elif ins == 'Dist':
                C_out = sorted(C_out, key=lambda i: self.d[s,i])
            elif ins == 's-TW':
                C_out = sorted(C_out, key=lambda i: self.e[i])
            elif ins == 'e-TW':
                C_out = sorted(C_out, key=lambda i: self.l[i])

            curr_solution = self.get_route_2nd(s, C_in, wc)
            curr_C = C_in.copy()

            for c_ in C_out:
                for i in range(len(C_in) + 1):
                    C_new = C_in[:i] + [c_] + C_in[i:]
                    new_solution = self.get_route_2nd(s, C_new, wc)

                    if new_solution[2] - new_solution[3] < curr_solution[2] - curr_solution[3]:
                        curr_solution, curr_C = new_solution, C_new.copy()

                C_in = curr_C.copy()

            C_out = [c for c in C_out if c not in C_in]

            # Save route
            if curr_solution[0] is not None and curr_solution[2] - curr_solution[3] < -1e-2:
                found_routes[tuple(curr_solution[0])] = curr_solution

            # Obtain best solution
            if curr_solution[2] - curr_solution[3] < best_solution[2] - best_solution[3]:
                dH = 1 - self.R
                best_solution = curr_solution
                not_improve = 0
            else:
                dH = -1 - self.R
                not_improve += 1

            # Update MAB
            for o in self.op:
                if o == (rem, ins):
                    self.H[o] += self.alpha*dH*(1-pi[o])
                else:
                    self.H[o] -= self.alpha*dH*pi[o]
                        
            self.n += 1
            self.R += dH/self.n

        return [(sol[0], sol[1], sol[2]) for sol in sorted(found_routes.values(), key=lambda sol: sol[2] - sol[3])[:10]]

    def set_partitioning_2nd(self, s, w):
        #Model
        sp_model = gp.Model(f"Subproblem: Set partitioning (Satellite {s})", env=self.env)
        sp_model.Params.MIPGap = 0

        assigned_c = {i for i in self.NC if w[i,s] > 0.9}
        
        key = (s, frozenset(assigned_c))
        price = key not in self.eval
        self.eval.add(key)        
        R_ = [r for r in self.routes[s] if self.cov[r,s] <= assigned_c]

        #Variables
        x = {r: sp_model.addVar(vtype = gp.GRB.CONTINUOUS, name = f"x^{s}({r})") for r in R_}
        y = {i: sp_model.addVar(vtype = gp.GRB.CONTINUOUS, name = f"a({i})") for i in self.NC}

        #Objective function
        Cmax = max(self.C.values())
        M = 10**(math.floor(math.log10(len(self.NC)*Cmax))+1)
        
        sp_model.setObjective(gp.quicksum(self.C[r,s]*x[r] for r in R_) + gp.quicksum(M*y[i] for i in self.NC), gp.GRB.MINIMIZE)

        #Constraints
        cover = {i: [] for i in self.NC}
        for r in R_:
            for i in self.cov[r,s]:
                cover[i].append(x[r])

        LocCtr_c = {}
        for i in self.NC:
            LocCtr_c[i] = sp_model.addConstr(gp.quicksum(cover[i]) == w[i,s] - y[i])

        Optimal = False if price else True

        while not Optimal and time.perf_counter() < self.deadline:
            sp_model.Params.TimeLimit = max(self.deadline - time.perf_counter(), 0.1)
            sp_model.update()
            sp_model.optimize()

            if sp_model.Status != gp.GRB.OPTIMAL:
                break

            Optimal = True
            
            wc = {i: LocCtr_c[i].Pi for i in self.NC}
            new_cols = self.ALNS_2nd(s, w, wc)

            for new_r, veh, cost in new_cols:
                r_id = f"r{len(self.routes[s])}"
                self.routes[s].append(r_id)
                self.Rd[r_id,s] = new_r
                self.vt[r_id,s] = veh
                self.C[r_id,s] = cost
                self.cov[r_id,s] = frozenset(i for i in new_r if i in self.NC)
                R_.append(r_id)
                Optimal = False

                newCol = gp.Column()
                newCol.addTerms([1.0]*len(self.cov[r_id,s]), [LocCtr_c[i] for i in self.cov[r_id,s]])

                x[r_id] = sp_model.addVar(vtype = gp.GRB.CONTINUOUS, obj=cost, name = f"x^{s}({r_id})", column = newCol)

        for r in R_:
            x[r].setAttr("vtype", gp.GRB.BINARY)

        sp_model.Params.TimeLimit = max(self.deadline - time.perf_counter(), 0.1)
        sp_model.update()
        sp_model.optimize()

        if sp_model.SolCount == 0:
            sp_model.dispose()
            return None, float("inf")

        if sum(y[i].X for i in self.NC) > 1e-2:
            sp_model.dispose()
            return None, float("inf")

        r_ = []
        for r in R_:
            if x[r].X > 0.9:
                r_.append(r)

        z_ = sp_model.ObjVal
        sp_model.dispose()
                
        return r_, z_

    def init_math(self, maxQ):
        #Model
        w_model = gp.Model("Initialize", env=self.env)

        #Variables
        w = {(i,s): w_model.addVar(vtype = gp.GRB.BINARY, name = f"w^{i}({s})") for i in self.NC for s in self.NS}

        #Objective function
        w_model.setObjective(gp.quicksum(self.d[s,i]*w[i,s] for i in self.NC for s in self.NS), gp.GRB.MINIMIZE)

        #Constraints
        for i in self.NC:
            w_model.addConstr(gp.quicksum(w[i,s] for s in self.NS) == 1) #Satellite - Customer

        for s in self.NS:
            w_model.addConstr(gp.quicksum(self.q[i]*w[i,s] for i in self.NC) <= maxQ) #Capacity

        w_model.Params.TimeLimit = max(self.deadline - time.perf_counter(), 0.1)
        w_model.update()
        w_model.optimize()

        if w_model.SolCount == 0:
            w_model.dispose()
            return None

        w_ = {(i,s): round(w[i,s].X) for s in self.NS for i in self.NC}
        w_model.dispose()
            
        return w_

    def matheuristic(self, exe_time):
        start = time.perf_counter()
        self.deadline = start + exe_time

        # Initialize routes
        # 1st level routes
        r = []
        for s in self.NS:
            for v in self.V1:
                rdx = len(r) if r else 0
                r_id = f"r{rdx}"
                    
                r.append(r_id)
                self.Rd[r_id,self.Nd] = [self.Nd,s,self.Nd]
                self.C[r_id,self.Nd] = self.h[v] + self.c[v]*(self.d[self.Nd,s] + self.d[s,self.Nd])
                self.vt[r_id,self.Nd] = v

        self.routes[self.Nd] = r

        # 2nd level routes
        for s in self.NS:
            r = []
            for i in self.NC:
                vt = [v for v in self.V2 if self.Q[v] >= self.q[i]]                

                for v in vt:
                    cost, order = self.elecPULSE([s,i,s],v)

                    if cost != float("inf"):
                        rdx = len(r) if r else 0
                        r_id = f"r{rdx}"

                        r.append(r_id)
                        self.Rd[r_id,s] = order
                        self.C[r_id,s] = cost
                        self.vt[r_id,s] = v
                        self.cov[r_id,s] = frozenset([i])

            self.routes[s] = r

        # Initialize w's
        maxQ = min(min(self.K.values()), max(self.Q.values()))
        w = self.init_math(maxQ)

        if w is None:
            return None, float("inf"), time.perf_counter() - start

        # Set partitioning
        z = {}
        r = {}

        r[self.Nd], z[self.Nd] = self.set_partitioning_1st(w)

        for s in self.NS:
            r[s], z[s] = self.set_partitioning_2nd(s, w)

        best_r = r.copy()
        best_z = sum(z.values())

        def eval_changes(sats, w):
            new_r, new_z = {}, {}

            if time.perf_counter() < self.deadline:
                new_r[self.Nd], new_z[self.Nd] = self.set_partitioning_1st(w)
            else:
                new_r[self.Nd], new_z[self.Nd] = None, float("inf")
            
            for s in sats:
                if time.perf_counter() < self.deadline:
                    new_r[s], new_z[s] = self.set_partitioning_2nd(s, w)
                else:
                    new_r[s], new_z[s] = None, float("inf")

            return sum(new_z.values()) - sum(z[k] for k in new_z), new_r, new_z

        while time.perf_counter() < self.deadline:           
            sat = {i: None for i in self.NC}
            q = {s: 0 for s in self.NS}

            for s in self.NS:
                for i in self.NC:
                    if w[i,s] > 0.9:
                        sat[i] = s
                        q[s] += self.q[i]

            moves = []

            for ni, i in enumerate(self.NC):
                si = sat[i]
                for s in self.NS:
                    if s != si and q[s] + self.q[i] <= maxQ:
                        moves.append(({(i,s): 1, (i,si): 0}, [si, s]))

                for j in self.NC[ni+1:]:
                    sj = sat[j]
                    if si != sj and q[si] - self.q[i] + self.q[j] <= maxQ and q[sj] - self.q[j] + self.q[i] <= maxQ:
                        moves.append(({(i,si): 0, (i,sj): 1, (j,si): 1, (j,sj): 0}, [si, sj]))

            best_delta = -1e-6
            best_move = None

            for changes, sats in moves:
                if time.perf_counter() < self.deadline:
                    w_old = {k: w[k] for k in changes}
                    w.update(changes)
                    eva = eval_changes(sats, w)

                    if eva[0] < best_delta:
                        best_delta = eva[0]
                        best_move = w.copy(), eva[1], eva[2]

                    w.update(w_old)
                else:
                    break

            if best_move is None:
                break

            w, new_r, new_z = best_move
            r.update(new_r)
            z.update(new_z)

            if sum(z.values()) < best_z:
                best_r = r.copy()
                best_z = sum(z.values())
            else:
                break

        end = time.perf_counter()

        if best_z != float("inf"):
            x_ = []

            for s, r_s in best_r.items():
                for r in r_s:
                    v = self.vt[r,s]
                    x_.append({"Vehicle type": v, "Route": self.Rd[r,s]})
        else:
            x_ = None

        return x_, best_z, end - start