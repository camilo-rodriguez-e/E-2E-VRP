# Electric Two-Echelon Vehicle Routing Problem (e-2E-VRP)

Optimization model for the Electric Two-Echelon Vehicle Routing Problem, implemented with Gurobi and Python.

## Overview

The Electric Two-Echelon Vehicle Routing Problem (E-2E-VRP) models urban delivery in two stages: large vehicles move goods from a depot to satellites, and electric vehicles deliver to customers. It includes capacity and battery constraints, aiming to minimize total routing and energy costs.

## Project Structure

```
├── model.ipynb              # Main optimization model (Gurobi)
├── e-2e-vrp instances/      # Problem instances
│   ├── H1/                   # Hub configuration 1
│   ├── H2/                   # Hub configuration 2
│   └── H3/                   # Hub configuration 3
│       └── Small/            # Small-sized instances
└── README.md
```

## Instance Format

Each instance file (`.txt`) contains:

- Depot and satellite coordinates
- Truck parameters (number, capacity, cost)
- City freighter parameters (capacity, battery, energy consumption)
- Customer data (coordinates, demand, time windows, service time)
- Recharge station locations

## License

Academic use — Research in network flow optimization.
