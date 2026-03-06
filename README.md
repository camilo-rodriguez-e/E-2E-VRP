# Electric Two-Echelon Vehicle Routing Problem (e-2E-VRP)

Optimization model for the Electric Two-Echelon Vehicle Routing Problem, implemented with Gurobi and Python.

## Overview

This project addresses a two-echelon distribution network where:

- **First echelon:** Trucks deliver from the depot to satellites
- **Second echelon:** Electric city freighters deliver from satellites to customers

The model considers time windows, vehicle capacities, battery constraints, and recharge stations for electric vehicles.

## Project Structure

```
├── model.ipynb              # Main optimization model (Gurobi)
├── create_instances.ipynb    # Instance generator
├── e-2e-vrp instances/      # Problem instances
│   ├── H1/                   # Hub configuration 1
│   ├── H2/                   # Hub configuration 2
│   └── H3/                   # Hub configuration 3
│       └── Small/            # Small-sized instances
├── Instances.xlsx            # Instance metadata
└── README.md
```

## Requirements

- Python 3.x
- Gurobi (optimization solver)
- `gurobipy`, `matplotlib`, `networkx`, `numpy`, `openpyxl`, `pandas`

## Usage

1. **Generate instances:** Run `create_instances.ipynb` to create or update problem instances.
2. **Solve the model:** Open `model.ipynb` and run the cells to load an instance and solve the e-2E-VRP.

## Instance Format

Each instance file (`.txt`) contains:

- Depot and satellite coordinates
- Truck parameters (number, capacity, cost)
- City freighter parameters (capacity, battery, energy consumption)
- Customer data (coordinates, demand, time windows, service time)
- Recharge station locations

## License

Academic use — Doctoral research in network flow optimization.
