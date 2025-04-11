# 🛩️ The Flying Sidekick Traveling Salesman Problem (FSTSP)

This repository contains an implementation of a heuristic algorithm described in the paper *[Murray and Chu. "The flying sidekick traveling salesman problem: Optimization of drone-assisted parcel delivery"](https://doi.org/10.1016/j.trc.2015.03.005)*.

---

## 🚀 Overview

In this implementation, I uses Google OR-Tools as the TSP solver.

- TSP Solver: [Google OR-Tools](https://developers.google.com/optimization)
- Routing and Map Data: Based on real-world road networks using OSMnx.

---

## 🧪 How to Run

```bash
python3 main.py
```

### 🔧 Configuration

- Algorithm parameters can be adjusted in the `fstsp_heuristic.py` file.
- You can test with different data by modifying the `testproblem` variable inside `main.py`.

---

## 🧾 Test Cases

- Test datasets are adapted from [https://github.com/optimatorlab/mFSTSP](https://github.com/optimatorlab/mFSTSP/tree/master/Problems).
- You may also run the algorithm with a custom list of latitude, longitude, and parcel weight inputs.

---

## 📦 Requirements

- `python` (>=3.6)
- `osmnx`
- `networkx`
- `numpy`
- `pandas`
- `ortools`