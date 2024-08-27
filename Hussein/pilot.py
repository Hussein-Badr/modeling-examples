import gurobipy as gp
from gurobipy import GRB

# Define the sets and parameters
N = []  # Set of nodes
A = []  # Set of arcs
K = []  # Set of vehicles
R = []  # Set of requests

# Reading data from the instance file
with open('/Users/zeyadosama/Desktop/E-JUST/Papers/Data/PDPT/PDPT-R5-K2-T1/PDPT-R5-K2-T1-Q100-0.txt', 'r') as file:
    lines = file.readlines()

# Extracting basic information
nr, nv, nt, capacity = map(int, lines[0].split())
K = list(range(nv))  # Vehicle IDs
capacity_k = {k: capacity for k in K}  # Vehicle capacities

# Node information
node_info = {}
for i in range(1, nr + 1):
    node_info[f'p{i-1}'] = list(map(int, lines[i].split()[1:]))
for i in range(1, nr + 1):
    node_info[f'd{i-1}'] = list(map(int, lines[nr + i].split()[1:]))
    
# Origin and destination depots
for i in range(nv):
    node_info[f'o{i}'] = list(map(int, lines[2 * nr + 1 + i].split()[1:]))
    node_info[f'e{i}'] = list(map(int, lines[2 * nr + 1 + nv + i].split()[1:]))

# Transfer stations
for i in range(nt):
    node_info[f't{i}'] = list(map(int, lines[2 * nr + 1 + 2 * nv + i].split()[1:]))

# Create set of nodes N and arcs A
N = list(node_info.keys())
for i in N:
    for j in N:
        if i != j:
            A.append((i, j))

# Travel cost associated with arc (i, j) and vehicle k
# Here we use Euclidean distance as an example for travel cost
c_kij = {(k, i, j): ((node_info[i][0] - node_info[j][0]) ** 2 + (node_info[i][1] - node_info[j][1]) ** 2) ** 0.5 for k in K for i, j in A}

# Demand of each request
q_r = {}
for i in range(nr):
    q_r[f'p{i}'] = int(lines[1 + i].split()[5])
    q_r[f'd{i}'] = -q_r[f'p{i}']

# Gurobi model
model = gp.Model("Vehicle Routing Problem with Time Windows")

# Decision variables
x = model.addVars(K, A, vtype=GRB.BINARY, name="x")  # Vehicle k travels through arc (i, j)
y = model.addVars(R, K, A, vtype=GRB.BINARY, name="y")  # Request r is transported by vehicle k through arc (i, j)
z = model.addVars(K, A, vtype=GRB.BINARY, name="z")  # Node i precedes node j for vehicle k
s = model.addVars(R, T, K, K, vtype=GRB.BINARY, name="s")
# Objective function: Minimize total travel cost
model.setObjective(gp.quicksum(c_kij[k, i, j] * x[k, i, j] for k in K for i, j in A), GRB.MINIMIZE)

# Constraints
# Flow constraints, capacity constraints, etc., should be added here based on your problem definition

#Original model constraints: 

# Constraint(2.3.4): Each request is assigned to exactly one vehicle at its pickup location
for r in R:
    model.addConstr(gp.quicksum(gp.quicksum(y[p(r), j, k, r] for j in N if j != p(r)) for k in K) == 1, f"assign_request_to_vehicle_r{r}")

# Constraint(2.3.5): Each request is assigned to exactly one vehicle at its delivery location
for r in R:
    model.addConstr(gp.quicksum(gp.quicksum(y[i, d(r), k, r] for i in N if i != d(r)) for k in K) == 1, f"assign_request_to_vehicle_r{r}_delivery")

# Constraint(2.3.6): Balanced flow of requests through transfer stations
for r in R:
    for i in T:
        model.addConstr(gp.quicksum(gp.quicksum(y[i, j, k, r] for j in N if j != i) for k in K) - gp.quicksum(gp.quicksum(y[j, i, k, r] for j in N if j != i) for k in K) == 0, f"balanced_flow_r{r}_i{i}")

# Constraint(2.3.8): Request can only be transported on arcs where the vehicle is traveling
for i, j in A:
    for k in K:
        for r in R:
            model.addConstr(y[i, j, k, r] <= x[i, j, k], f"request_on_arc_constraint_{i}_{j}_{k}_{r}")

# Constraint(2.3.9): Vehicle capacity constraint
for i, j in A:
    for k in K:
        model.addConstr(gp.quicksum(q_r * y[i, j, k, r] for r in R) <= u_k * x[i, j, k], f"vehicle_capacity_constraint_{i}_{j}_{k}")


# Constraint(2.3.16): Request flows cannot include nodes that are transfer stations, pickup, or delivery locations
for r in R:
    for k in K:
        for i in N:
            if i not in T and i != p(r) and i != d(r):
                model.addConstr(gp.quicksum(y[i, j, k, r] for j in N if j != i) - gp.quicksum(y[j, i, k, r] for j in N if j != i) == 0, f"no_direct_transport_through_non_relevant_nodes_r{r}_k{k}_i{i}")

# Constraint(2.3.21): A request can only be transported by a single vehicle between pickup and delivery, unless it's being transferred
for r in R:
    for t in T:
        for k1 in K:
            for k2 in K:
                if k1 != k2:
                    model.addConstr(gp.quicksum(gp.quicksum(y[i, j, k1, r] for j in N if j != i) for i in N if i != p(r)) + gp.quicksum(gp.quicksum(y[j, t, k2, r] for j in N if j != t) for i in N if i != d(r)) <= 1 + s[t, j, k1, k2], f"single_vehicle_transport_r{r}_t{t}_k1{k1}_k2{k2}")

# Constraint(2.3.25): Vehicle can only leave its origin depot once
for k in K:
    model.addConstr(gp.quicksum(x[k, o(k), j] for j in N if j != o(k)) == 1, f"leave_origin_once_k{k}")

# Constraint(2.4.2): Balanced flow of vehicles through nodes
for k in K:
    for i in PUDUT:
        model.addConstr(gp.quicksum(gp.quicksum(x[i, j, k] for j in N if j != i) for j in N) - gp.quicksum(gp.quicksum(x[j, i, k] for j in N if j != i) for j in N) == 0, f"balanced_flow_vehicles_k{k}_i{i}")

# Constraint(2.4.8): Non-negativity of arrival and departure times
for i in N:
    for k in K:
        model.addConstr(a[i, k] >= 0, f"non_negative_arrival_time_{i}_{k}")
        model.addConstr(b[i, k] >= 0, f"non_negative_departure_time_{i}_{k}")
#added resundent constraints
# Constraint(2.5.1): Vehicle cannot return to its origin depot
for k in K:
    model.addConstr(gp.quicksum(x[k, j, o(k)] for j in N if j != o(k)) == 0, f"no_return_to_origin_k{k}")

# Constraint(2.5.2): Vehicle cannot visit nodes other than its origin and destination depots
for k in K:
    for i in N:
        if i not in [o(k), 'o (k)]:
            model.addConstr(gp.quicksum(x[k, j, i] for j in N) == 0, f"no_visit_other_nodes_k{k}_i{i}")

# Constraint(2.5.3): Vehicle can only enter its destination depot once
for k in K:
    model.addConstr(gp.quicksum(x[k, j, 'o(k)] for j in N if j != 'o(k)) == 1, f"enter_destination_once_k{k}")

# Constraint(2.5.4): Vehicle cannot return to its origin depot
for k in K:
    model.addConstr(gp.quicksum(x[k, j, 'o(k)] for j in N if j != 'o(k)) == 0, f"no_return_to_origin_k{k}")

# Constraint(2.5.5): Vehicle can visit a transfer station at most once
for k in K:
    for i in T:
        model.addConstr(gp.quicksum(x[k, j, i] for j in N if j != i) <= 1, f"visit_transfer_station_once_k{k}_i{i}")

# Constraint(5.5.6): Each pickup or delivery location is visited exactly once
for i in PUD:
    model.addConstr(gp.quicksum(gp.quicksum(x[k, j, i] for j in N if j != i) for k in K) == 1, f"visit_pickup_delivery_once_i{i}")

# Constraint(2.5.7): Request cannot be directly transported to its pickup location
for r in R:
    model.addConstr(gp.quicksum(gp.quicksum(y[i, j, k, r] for i in N if i != p(r)) for k in K) == 0, f"no_direct_transport_to_pickup_r{r}")

# Constraint(2.5.8): Request flows cannot include origin or destination depots
for r in R:
    for k in K:
        for i in OUD:
            model.addConstr(gp.quicksum(y[i, j, k, r] for j in N if j != i) == 0, f"no_direct_transport_through_origin_destination_r{r}_k{k}_i{i}")

# Constraint(2.5.9): Ensure vehicle k1 arrives before vehicle k2 for request r at transfer station t
for r in R:
    for t in T:
        for k1 in K:
            for k2 in K:
                if k1 != k2:
                    model.addConstr(a[t, k1] - b[t, k2] <= M * (1 - s[r, t, k1, k2]), f"precedence_r{r}_t{t}_k1{k1}_k2{k2}")

# Constraint(2.5.10): Time window constraints
for i, j in A:
    for k in K:
        model.addConstr(b[i, k] + t_ij^k - a[j, k] <= M * (1 - x[i, j, k]), f"time_window_constraint_{i}_{j}_{k}")

# Constraint(2.5.11): Time window constraints for arrival and departure
for i in N:
    for k in K:
        model.addConstr(a[i, k] >= E[i], f"arrival_time_constraint_{i}_{k}")
        model.addConstr(b[i, k] <= L[i], f"departure_time_constraint_{i}_{k}")

# Constraint(2.5.12): Arrival time must be less than or equal to departure time
for i in N:
    for k in K:
        model.addConstr(a[i, k] <= b[i, k], f"arrival_before_departure_{i}_{k}")


# Optimize the model
model.optimize()

# Display results
if model.status == GRB.OPTIMAL:
    print("Optimal solution found:")
    for k in K:
        for i, j in A:
            if x[k, i, j].X > 0.5:
                print(f"Vehicle {k} travels from {i} to {j}")

else:
    print("No optimal solution found.")