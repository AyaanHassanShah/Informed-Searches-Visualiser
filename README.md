# Dynamic Pathfinding Agent

**FAST-NUCES Assignment** — Informed Search Algorithms on Dynamic Grids

A comprehensive interactive visualization of pathfinding algorithms with real-time obstacle dynamics and performance metrics.

---

## 📌 Overview

This project implements and compares two informed search algorithms (**A*** and **GBFS**) on a dynamic grid environment. The GUI allows interactive testing, configuration, and visualization of pathfinding behavior under various conditions.

**Key Features:**
- ✅ **A* Search** with optimality guarantee
- ✅ **Greedy Best-First Search (GBFS)** with speed focus
- ✅ **Multiple Heuristics**: Manhattan & Euclidean distance
- ✅ **Dynamic Mode**: Real-time obstacle spawning with automatic re-planning
- ✅ **Interactive GUI**: Click-to-toggle obstacles, drag sliders for live adjustments
- ✅ **Performance Metrics**: Nodes visited, execution time, re-plan count
- ✅ **Scenario Modes**: Best-case and worst-case demonstration scenarios

---

## 🚀 Installation

### Requirements
- Python 3.8+
- numpy
- matplotlib

### Setup

```bash
# Navigate to project directory
cd "d:\Fifth Semester\Sixth Semester\AI-A2-23F-0711"

# Install dependencies
pip install numpy matplotlib

# Run the application
python dynamic_pathfinding.py
```

---

## 🎮 User Controls

### Grid Interaction
| Control | Action |
|---------|--------|
| **Click Cell** | Toggle obstacle on/off |
| **Set Start 🟢** | Click button → click cell to place start node |
| **Set Goal 🔴** | Click button → click cell to place goal node |

### Algorithm Configuration
- **Algorithm**: Select A* or GBFS
- **Heuristic**: Choose Manhattan or Euclidean distance
- **Grid Size**: Adjust rows × columns (5–40)
- **Obstacle Density**: Control random obstacle percentage

### Execution Modes
- **Search**: Run one-time pathfinding with animation
- **Dynamic: OFF/ON**: Toggle live obstacle spawning & re-planning
- **Stop**: Halt current execution

### Dynamic Mode Controls
- **Spawn Probability**: Likelihood of new obstacle appearing each step
- **Animation Speed**: Slow down/speed up visualization

---

## 🔍 Algorithm Details

### A* Search
- **Formula**: `f(n) = g(n) + h(n)`
  - `g(n)` = cost from start to node n
  - `h(n)` = heuristic estimate to goal
- **Guarantee**: Finds shortest path (if heuristic is admissible)
- **Trade-off**: More nodes explored, guaranteed optimality

### Greedy Best-First Search (GBFS)
- **Formula**: `f(n) = h(n)` only
- **Advantage**: Faster exploration
- **Trade-off**: May not find shortest path (not optimal)

### Heuristics
- **Manhattan**: `|x₁ - x₂| + |y₁ - y₂|` (grid-based)
- **Euclidean**: `√[(x₁ - x₂)² + (y₁ - y₂)²]` (straight-line distance)

---

## 📊 Metrics Dashboard

| Metric | Description |
|--------|-------------|
| **Algorithm** | Currently selected search method |
| **Heuristic** | Distance estimation function being used |
| **Nodes Visited** | Total cells explored during search |
| **Path Cost** | Length of final path (moves to goal) |
| **Exec Time** | Milliseconds to find path |
| **Re-plans** | Number of re-calculations triggered by obstacles (dynamic mode) |
| **Status** | Current operation state & results |

---

## 🗺️ Color Legend

| Color | Meaning |
|-------|---------|
| 🟢 Green | Start node |
| 🔴 Red | Goal node |
| 🟦 Blue | Visited / explored cells |
| 🟨 Yellow | Frontier / pending cells |
| 🟩 Bright Green | Final path |
| 🟠 Orange | Current agent position |
| ⬛ Dark Gray | Obstacles / blocked cells |
| ⬜ Light Gray | Empty / free cells |

---

## 🎯 Usage Scenarios

### Scenario 1: Compare Algorithm Performance
1. Click **"Random Map"** to generate obstacles
2. Select algorithm (A* vs GBFS)
3. Click **"Search"** and observe:
   - Which algorithm explores fewer nodes?
   - Which finds the shorter path?
   - Execution time differences?

### Scenario 2: Test Heuristic Impact
1. Generate a map
2. Run with **Manhattan** heuristic → note metrics
3. Run with **Euclidean** heuristic → compare results

### Scenario 3: Dynamic Replanning
1. Enable **"Dynamic: ON"**
2. Adjust **"Spawn Probability"** slider
3. Click **"Search"** to watch agent navigate while obstacles spawn
4. Observe re-planning triggers and path adjustments

### Scenario 4: Best/Worst Case Analysis
1. **Best Case**: Few obstacles, clear direct path → minimal nodes visited
2. **Worst Case**: Dense maze → maximum exploration needed

---

## 📁 Project Structure

```
AI-A2-23F-0711/
├── dynamic_pathfinding.py   # Main application (all code)
└── README.md                # This file
```

### Code Organization

```python
# Heuristic Functions
manhattan(a, b)             # Grid distance
euclidean(a, b)             # Straight-line distance

# Search Algorithms
astar(env, start, goal, hfn)         # A* implementation
gbfs(env, start, goal, hfn)          # GBFS implementation

# Grid Environment
class GridEnvironment:
  - toggle(r, c)                     # Toggle obstacle
  - random_map(density)              # Generate random obstacles
  - spawn_obstacle(prob)             # Dynamic obstacle creation
  - neighbors(r, c)                  # Get free adjacent cells

# GUI
class PathfindingGUI:
  - _build_figure()                  # Create interface
  - draw_grid()                      # Render visualization
  - _run_static()                    # One-time search
  - _run_dynamic()                   # Dynamic mode with replanning
```

---

## 🧪 Example: Running A* vs GBFS

```python
# Both algorithms return: (path, visited_cells, frontier_cells, nodes_visited, time_ms)

path, visited, frontier, nv, ms = astar(env, start, goal, manhattan)
print(f"A* with Manhattan: {nv} nodes explored, {ms:.2f}ms, path length: {len(path)-1}")

path, visited, frontier, nv, ms = gbfs(env, start, goal, euclidean)
print(f"GBFS with Euclidean: {nv} nodes explored, {ms:.2f}ms, path length: {len(path)-1}")
```

---

## 💡 Key Insights

### When to Use A*?
✅ Need **guaranteed shortest path**
✅ Small to medium grids (performance acceptable)
✅ Accurate heuristic available

### When to Use GBFS?
✅ Need **fast approximate solution**
✅ Large grids (speed critical)
✅ Optimality not required

### Heuristic Selection
- **Manhattan**: Faster, suitable for 4-directional movement
- **Euclidean**: More accurate, 8-directional movement friendly

---

## 📈 Performance Expectations

| Grid Size | Density | Algorithm | Nodes (Avg) | Time (ms) |
|-----------|---------|-----------|------------|-----------|
| 20×20 | 30% | A* + Manhattan | 80–150 | 0.5–1.5 |
| 20×20 | 30% | GBFS + Manhattan | 40–100 | 0.2–0.8 |
| 40×40 | 20% | A* + Euclidean | 400–800 | 2–5 |
| 40×40 | 20% | GBFS + Euclidean | 200–500 | 1–3 |

*Actual performance varies by obstacle configuration and start/goal positions*

---

## 🔧 Customization

### Adjusting Grid Parameters
Use sliders in GUI to modify:
- Grid dimensions (5–40 cells)
- Obstacle density (0–70%)
- Spawn probability (1–30%)
- Animation speed (0.02–0.60 sec/step)

### Modifying Heuristic Functions
Edit `manhattan()` or `euclidean()` functions to test custom distance metrics

### Scenario Creation
Edit `GridEnvironment.random_map(density)` or add new methods for custom obstacle patterns

---

## 📚 References

- **A* Search**: Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). "A Formal Basis for the Heuristic Determination of Minimum Cost Paths"
- **Greedy Best-First**: Pearl, J. (1984). "Heuristics: Intelligent Search Strategies for Computer Problem Solving"
- **Pathfinding in Games**: Buckland, M. (2005). "Game Development with Lua"

---

## 📝 Notes

- **Dynamic Mode**: Obstacles spawn at random positions; if they block the planned path, a full re-plan is triggered from the agent's current position
- **Tie-Breaking**: Counter mechanism ensures deterministic behavior when multiple cells have identical f(n) scores
- **Memory**: Grid stored as NumPy array for efficient operations
- **Visualization**: Matplotlib drawn incrementally to show search progression

---

## 👨‍💻 Author

Created for FAST-NUCES | Artificial Intelligence Course (Semester 6)

---

## 📄 License

Educational use. FAST-NUCES Property.

