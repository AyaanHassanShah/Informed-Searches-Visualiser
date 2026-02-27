"""
Dynamic Pathfinding Agent
=========================
FAST-NUCES Assignment — Informed Search Algorithms
Implements A*, GBFS with Manhattan/Euclidean heuristics on a dynamic grid.
GUI built with Matplotlib + Widgets (no Tkinter / Pygame required).

Controls:
  • Click grid cells   → toggle obstacle
  • Set Start/Goal     → click button, then click cell
  • Search             → runs selected algorithm
  • Dynamic Mode ON    → agent animates with live obstacle spawning + auto re-planning
  • Sliders            → grid size, density, spawn probability, speed
"""

import math
import heapq
import random
import time
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.widgets import Button, Slider, RadioButtons

matplotlib.rcParams['toolbar'] = 'None'

# ─────────────────────────────────────────────
#  Heuristic Functions
# ─────────────────────────────────────────────
def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def euclidean(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


# ─────────────────────────────────────────────
#  Search Algorithms
# ─────────────────────────────────────────────
def astar(env, start, goal, hfn):
    """A* Search — f(n) = g(n) + h(n)"""
    counter = 0  # tie-breaker
    open_heap = []
    heapq.heappush(open_heap, (hfn(start, goal), counter, 0, start))
    came_from = {start: None}
    g_score   = {start: 0}
    visited   = set()
    frontier  = {start}
    nodes_visited = 0

    while open_heap:
        f, _, g, cur = heapq.heappop(open_heap)
        if cur in visited:
            continue
        visited.add(cur)
        frontier.discard(cur)
        nodes_visited += 1

        if cur == goal:
            return _reconstruct(came_from, goal), visited, frontier, nodes_visited

        for nb in env.neighbors(*cur):
            if nb in visited:
                continue
            tentative_g = g + 1
            if nb not in g_score or tentative_g < g_score[nb]:
                g_score[nb]  = tentative_g
                came_from[nb] = cur
                counter += 1
                heapq.heappush(open_heap,
                    (tentative_g + hfn(nb, goal), counter, tentative_g, nb))
                frontier.add(nb)

    return None, visited, frontier, nodes_visited


def gbfs(env, start, goal, hfn):
    """Greedy Best-First Search — f(n) = h(n)"""
    counter = 0
    open_heap = []
    heapq.heappush(open_heap, (hfn(start, goal), counter, start))
    came_from = {start: None}
    visited   = set()
    frontier  = {start}
    nodes_visited = 0

    while open_heap:
        h, _, cur = heapq.heappop(open_heap)
        if cur in visited:
            continue
        visited.add(cur)
        frontier.discard(cur)
        nodes_visited += 1

        if cur == goal:
            return _reconstruct(came_from, goal), visited, frontier, nodes_visited

        for nb in env.neighbors(*cur):
            if nb not in visited and nb not in came_from:
                came_from[nb] = cur
                counter += 1
                heapq.heappush(open_heap, (hfn(nb, goal), counter, nb))
                frontier.add(nb)

    return None, visited, frontier, nodes_visited


def _reconstruct(came_from, goal):
    path, node = [], goal
    while node is not None:
        path.append(node)
        node = came_from[node]
    return list(reversed(path))


# ─────────────────────────────────────────────
#  Grid Environment
# ─────────────────────────────────────────────
class GridEnvironment:
    def __init__(self, rows=20, cols=20):
        self.rows  = rows
        self.cols  = cols
        self.grid  = np.zeros((rows, cols), dtype=np.int8)
        self.start = (0, 0)
        self.goal  = (rows - 1, cols - 1)

    # ── obstacle helpers ──────────────────────
    def toggle(self, r, c):
        if self._valid_cell(r, c):
            self.grid[r, c] ^= 1

    def set_obs(self, r, c, val=1):
        if self._valid_cell(r, c):
            self.grid[r, c] = val

    def _valid_cell(self, r, c):
        return (0 <= r < self.rows and 0 <= c < self.cols
                and (r, c) != self.start and (r, c) != self.goal)

    def clear(self):
        self.grid[:] = 0

    def random_map(self, density=0.30):
        self.clear()
        for r in range(self.rows):
            for c in range(self.cols):
                if (r, c) not in (self.start, self.goal):
                    if random.random() < density:
                        self.grid[r, c] = 1

    # ── navigation ───────────────────────────
    def is_free(self, r, c):
        return 0 <= r < self.rows and 0 <= c < self.cols and self.grid[r, c] == 0

    def neighbors(self, r, c):
        for dr, dc in ((-1,0),(1,0),(0,-1),(0,1)):
            nr, nc = r + dr, c + dc
            if self.is_free(nr, nc):
                yield (nr, nc)

    # ── dynamic obstacle ─────────────────────
    def spawn_obstacle(self, prob=0.05):
        if random.random() >= prob:
            return None
        empties = [(r, c) for r in range(self.rows) for c in range(self.cols)
                   if self.grid[r, c] == 0
                   and (r, c) not in (self.start, self.goal)]
        if not empties:
            return None
        r, c = random.choice(empties)
        self.grid[r, c] = 1
        return (r, c)

    def resize(self, rows, cols):
        self.rows  = rows
        self.cols  = cols
        self.grid  = np.zeros((rows, cols), dtype=np.int8)
        self.start = (0, 0)
        self.goal  = (rows - 1, cols - 1)


# ─────────────────────────────────────────────
#  Color Palette
# ─────────────────────────────────────────────
C = dict(
    empty    = np.array([0.95, 0.95, 1.00]),
    obstacle = np.array([0.17, 0.17, 0.17]),
    visited  = np.array([0.40, 0.60, 1.00]),
    frontier = np.array([1.00, 0.87, 0.00]),
    path     = np.array([0.00, 0.80, 0.27]),
    start    = np.array([0.00, 0.67, 0.00]),
    goal     = np.array([0.80, 0.00, 0.00]),
    agent    = np.array([1.00, 0.40, 0.00]),
)


# ─────────────────────────────────────────────
#  GUI
# ─────────────────────────────────────────────
class PathfindingGUI:
    def __init__(self):
        self.env           = GridEnvironment(20, 20)
        self.algorithm     = 'A*'
        self.heuristic     = 'Manhattan'
        self.dynamic_mode  = False
        self.is_running    = False
        self.setting_start = False
        self.setting_goal  = False
        self._replans      = 0

        self._build_figure()
        self.draw_grid()
        plt.show()

    # ─── Figure Layout ───────────────────────
    def _build_figure(self):
        self.fig = plt.figure(figsize=(17, 10), facecolor='#1a1a2e')
        self.fig.canvas.manager.set_window_title('Dynamic Pathfinding Agent — FAST NUCES')

        # Grid axes (left 63%)
        self.ax = self.fig.add_axes([0.01, 0.04, 0.62, 0.93])
        self.ax.set_facecolor('#16213e')

        X = 0.66   # control panel left edge
        W = 0.31   # control panel width

        def label(y, text, size=10, color='#aaaaff', bold=False):
            self.fig.text(X, y, text, fontsize=size, color=color, fontweight='bold' if bold else 'normal')

        # ── Title ──
        self.fig.text(X + W/2, 0.965, 'Dynamic Pathfinding Agent',
                      fontsize=13, fontweight='bold', color='white', ha='center')
        self.fig.text(X + W/2, 0.945, 'FAST-NUCES | Informed Search',
                      fontsize=9, color='#888888', ha='center')

        # ── Grid Size ──
        label(0.915, '● Grid Size (Rows × Cols)')
        ax_r = self.fig.add_axes([X,        0.875, 0.145, 0.026])
        ax_c = self.fig.add_axes([X + 0.16, 0.875, 0.145, 0.026])
        self.sl_rows = self._slider(ax_r, 'Rows', 5, 40, 20)
        self.sl_cols = self._slider(ax_c, 'Cols', 5, 40, 20)

        # ── Obstacle Density ──
        label(0.845, '● Obstacle Density')
        ax_d = self.fig.add_axes([X, 0.810, 0.31, 0.026])
        self.sl_density = self._slider(ax_d, 'Density', 0.0, 0.7, 0.30)

        # ── Algorithm ──
        label(0.788, '● Algorithm')
        ax_al = self.fig.add_axes([X, 0.720, 0.13, 0.060])
        ax_al.set_facecolor('#0f3460')
        self.rb_algo = RadioButtons(ax_al, ('A*', 'GBFS'), activecolor='#e94560')
        self._style_radio(self.rb_algo)

        # ── Heuristic ──
        self.fig.text(X + 0.16, 0.788, '● Heuristic', fontsize=10, color='#aaaaff', fontweight='bold')
        ax_h = self.fig.add_axes([X + 0.16, 0.720, 0.155, 0.060])
        ax_h.set_facecolor('#0f3460')
        self.rb_heur = RadioButtons(ax_h, ('Manhattan', 'Euclidean'), activecolor='#e94560')
        self._style_radio(self.rb_heur)

        # ── Action Buttons ──
        BW, BH, BY = 0.145, 0.038, [0.670, 0.622, 0.574, 0.526]
        self.btn_resize    = self._btn([X,        BY[0], BW, BH], 'Resize Grid')
        self.btn_random    = self._btn([X+0.16,   BY[0], BW, BH], 'Random Map')
        self.btn_clear     = self._btn([X,        BY[1], BW, BH], 'Clear Grid')
        self.btn_setstart  = self._btn([X+0.16,   BY[1], BW, BH], 'Set Start 🟢')
        self.btn_setgoal   = self._btn([X,        BY[2], BW, BH], 'Set Goal 🔴')
        self.btn_search    = self._btn([X+0.16,   BY[2], BW, BH], '▶  Search',
                                       fc='#006400', hc='#00aa00')
        self.btn_dynamic   = self._btn([X,        BY[3], BW, BH], 'Dynamic: OFF')
        self.btn_stop      = self._btn([X+0.16,   BY[3], BW, BH], '■  Stop',
                                       fc='#8b0000', hc='#cc0000')

        # ── Dynamic Controls ──
        label(0.500, '● Spawn Probability (Dynamic Mode)')
        ax_sp = self.fig.add_axes([X, 0.465, 0.31, 0.026])
        self.sl_spawn = self._slider(ax_sp, 'Prob', 0.01, 0.30, 0.05)

        label(0.437, '● Animation Speed')
        ax_sv = self.fig.add_axes([X, 0.402, 0.31, 0.026])
        self.sl_speed = self._slider(ax_sv, 'Speed', 0.02, 0.60, 0.12)

        # ── Metrics Dashboard ──
        label(0.370, '── Metrics Dashboard ──', color='#e94560', bold=True)
        self.t_algo    = self.fig.text(X, 0.342, 'Algorithm : A*',           fontsize=9, color='#ccccff')
        self.t_heur    = self.fig.text(X, 0.318, 'Heuristic : Manhattan',    fontsize=9, color='#ccccff')
        self.t_nodes   = self.fig.text(X, 0.294, 'Nodes Visited : —',        fontsize=9, color='#ccccff')
        self.t_cost    = self.fig.text(X, 0.270, 'Path Cost : —',            fontsize=9, color='#ccccff')
        self.t_time    = self.fig.text(X, 0.246, 'Exec Time : —',            fontsize=9, color='#ccccff')
        self.t_replans = self.fig.text(X, 0.222, 'Re-plans : 0',             fontsize=9, color='#ccccff')
        self.t_status  = self.fig.text(X, 0.192, 'Status : Ready',           fontsize=10,
                                        color='#44ff44', fontweight='bold')

        # ── Legend ──
        label(0.158, '── Legend ──', color='#e94560', bold=True)
        items = [('Start','#00aa00'),('Goal','#cc0000'),('Frontier','#ffdd00'),
                 ('Visited','#6699ff'),('Path','#00cc44'),('Agent','#ff6600'),('Obstacle','#444444')]
        for i, (lbl, col) in enumerate(items):
            dx = (i % 2) * 0.17
            dy = (i // 2) * 0.024
            self.fig.text(X + dx,        0.134 - dy, '■', color=col, fontsize=13)
            self.fig.text(X + dx + 0.018,0.134 - dy, lbl, color='white', fontsize=8)

        # ── Connect Callbacks ──
        self.rb_algo.on_clicked(self._cb_algo)
        self.rb_heur.on_clicked(self._cb_heur)
        self.btn_resize.on_clicked(self._cb_resize)
        self.btn_random.on_clicked(self._cb_random)
        self.btn_clear.on_clicked(self._cb_clear)
        self.btn_setstart.on_clicked(self._cb_setstart)
        self.btn_setgoal.on_clicked(self._cb_setgoal)
        self.btn_search.on_clicked(self._cb_search)
        self.btn_dynamic.on_clicked(self._cb_dynamic)
        self.btn_stop.on_clicked(self._cb_stop)
        self.fig.canvas.mpl_connect('button_press_event', self._cb_click)

    def _slider(self, ax, label, vmin, vmax, vinit):
        sl = Slider(ax, label, vmin, vmax, valinit=vinit, color='#3a3aaa')
        sl.label.set_color('white');  sl.label.set_fontsize(8)
        sl.valtext.set_color('white');sl.valtext.set_fontsize(8)
        ax.set_facecolor('#0f3460')
        return sl

    def _btn(self, rect, label, fc='#0f3460', hc='#e94560'):
        ax  = self.fig.add_axes(rect)
        btn = Button(ax, label, color=fc, hovercolor=hc)
        btn.label.set_color('white');  btn.label.set_fontsize(9)
        return btn

    def _style_radio(self, rb):
        for lbl in rb.labels:
            lbl.set_color('white');  lbl.set_fontsize(9)

    # ─── Drawing ─────────────────────────────
    def draw_grid(self, visited=None, frontier=None, path=None, agent=None):
        self.ax.clear()
        rows, cols = self.env.rows, self.env.cols

        img = np.tile(C['empty'], (rows, cols, 1)).astype(float)

        for r in range(rows):
            for c in range(cols):
                if self.env.grid[r, c]:
                    img[r, c] = C['obstacle']

        if visited:
            for (r, c) in visited:
                if (r, c) not in (self.env.start, self.env.goal):
                    img[r, c] = C['visited']

        if frontier:
            for (r, c) in frontier:
                if (r, c) not in (self.env.start, self.env.goal):
                    img[r, c] = C['frontier']

        if path:
            for (r, c) in path:
                if (r, c) not in (self.env.start, self.env.goal):
                    img[r, c] = C['path']

        sr, sc = self.env.start
        gr, gc = self.env.goal
        img[sr, sc] = C['start']
        img[gr, gc] = C['goal']

        if agent and agent not in (self.env.start, self.env.goal):
            img[agent[0], agent[1]] = C['agent']

        self.ax.imshow(img, aspect='equal', interpolation='nearest')

        # grid lines
        lw = max(0.2, 0.8 - rows * 0.02)
        for r in range(rows + 1):
            self.ax.axhline(r - 0.5, color='#555577', lw=lw, alpha=0.5)
        for c in range(cols + 1):
            self.ax.axvline(c - 0.5, color='#555577', lw=lw, alpha=0.5)

        fs = max(5, min(14, 200 // max(rows, cols)))
        self.ax.text(sc, sr, 'S', ha='center', va='center',
                     fontsize=fs, color='white', fontweight='bold')
        self.ax.text(gc, gr, 'G', ha='center', va='center',
                     fontsize=fs, color='white', fontweight='bold')

        if agent and agent not in (self.env.start, self.env.goal):
            self.ax.text(agent[1], agent[0], '●', ha='center', va='center',
                         fontsize=max(6, fs - 2), color='white')

        self.ax.set_xlim(-0.5, cols - 0.5)
        self.ax.set_ylim(rows - 0.5, -0.5)
        self.ax.set_xticks([]); self.ax.set_yticks([])
        mode_str = ' | DYNAMIC MODE' if self.dynamic_mode else ''
        self.ax.set_title(
            f'Grid {rows}×{cols}{mode_str}  |  Click to toggle obstacles  |  '
            f'Alg: {self.algorithm}  |  Heuristic: {self.heuristic}',
            color='white', fontsize=9, pad=6)
        self.fig.canvas.draw_idle()

    # ─── Metrics Helpers ─────────────────────
    def _set_status(self, text, color=None):
        self.t_status.set_text(f'Status : {text}')
        if color:
            self.t_status.set_color(color)
        elif 'Found' in text or 'Reached' in text or 'Ready' in text:
            self.t_status.set_color('#44ff44')
        elif 'No path' in text or 'Blocked' in text or 'Stop' in text:
            self.t_status.set_color('#ff4444')
        else:
            self.t_status.set_color('#ffaa00')

    def _update(self, nodes=None, cost=None, ms=None, status=None, replans=None):
        if nodes   is not None: self.t_nodes.set_text(f'Nodes Visited : {nodes}')
        if cost    is not None: self.t_cost.set_text(f'Path Cost : {cost}')
        if ms      is not None: self.t_time.set_text(f'Exec Time : {ms:.2f} ms')
        if status  is not None: self._set_status(status)
        if replans is not None: self.t_replans.set_text(f'Re-plans : {replans}')

    # ─── Search Runner ───────────────────────
    def _search(self, start=None, goal=None):
        start = start or self.env.start
        goal  = goal  or self.env.goal
        hfn   = manhattan if self.heuristic == 'Manhattan' else euclidean
        t0    = time.perf_counter()
        if self.algorithm == 'A*':
            path, vis, front, nv = astar(self.env, start, goal, hfn)
        else:
            path, vis, front, nv = gbfs(self.env, start, goal, hfn)
        ms = (time.perf_counter() - t0) * 1000
        return path, vis, front, nv, ms

    # ─── Static Search ───────────────────────
    def _run_static(self):
        self._set_status('Searching…', '#ffaa00')
        self.fig.canvas.draw_idle()

        path, vis, front, nv, ms = self._search()
        self._update(nodes=nv, ms=ms)

        if path is None:
            self._update(cost='N/A', status='No path found!')
            self.draw_grid(visited=vis)
            return

        self._update(cost=len(path) - 1, status=f'Path Found  (len={len(path)-1})')

        speed = self.sl_speed.val
        vis_list = list(vis)
        chunk    = max(1, len(vis_list) // 25)

        # Animate visited expansion
        for i in range(0, len(vis_list), chunk):
            if not plt.fignum_exists(self.fig.number): return
            self.draw_grid(visited=set(vis_list[:i + chunk]), frontier=front)
            plt.pause(speed * 0.25)

        # Show final path
        self.draw_grid(visited=vis, path=path)

    # ─── Dynamic Search ──────────────────────
    def _run_dynamic(self):
        self.is_running = True
        self._replans   = 0
        self._update(replans=0, status='Planning initial path…')
        self.fig.canvas.draw_idle()

        path, vis, front, nv, ms = self._search()
        total_nv, total_ms = nv, ms

        if path is None:
            self._update(nodes=nv, ms=ms, cost='N/A', status='No path found!')
            self.is_running = False
            return

        self._update(nodes=nv, cost=len(path) - 1, ms=ms, status='Agent moving…')

        agent     = self.env.start
        pidx      = 1         # next step index in path
        speed     = self.sl_speed.val

        while self.is_running and agent != self.env.goal:
            if not plt.fignum_exists(self.fig.number): break

            speed = self.sl_speed.val   # allow live adjustment

            # Try spawning a new obstacle
            new_obs = self.env.spawn_obstacle(self.sl_spawn.val)

            # Re-plan if new obstacle blocks remaining path
            if new_obs is not None and path and new_obs in path[pidx:]:
                self._replans += 1
                self._update(status=f'Re-planning (#{self._replans})…', replans=self._replans)

                path, vis, front, nv, ms = self._search(start=agent)
                total_nv += nv;  total_ms += ms

                if path is None:
                    self._update(nodes=total_nv, ms=total_ms,
                                 status='Path blocked — no route!', cost='N/A')
                    self.draw_grid(visited=vis, agent=agent)
                    break

                pidx = 1
                self._update(nodes=total_nv, ms=total_ms,
                             cost=len(path) - 1, replans=self._replans)

            # Advance agent
            if path and pidx < len(path):
                agent = path[pidx]
                pidx += 1

            remaining = len(path) - pidx if path else 0
            self._update(cost=remaining)
            self.draw_grid(visited=vis, path=path, agent=agent)
            plt.pause(speed)

        if agent == self.env.goal:
            self._update(status='Goal Reached ✓', replans=self._replans)
            self.draw_grid(visited=vis, path=path)

        self.is_running = False

    # ─── Callbacks ───────────────────────────
    def _cb_algo(self, label):
        self.algorithm = label
        self.t_algo.set_text(f'Algorithm : {label}')

    def _cb_heur(self, label):
        self.heuristic = label
        self.t_heur.set_text(f'Heuristic : {label}')

    def _cb_resize(self, _):
        r = int(self.sl_rows.val); c = int(self.sl_cols.val)
        self.env.resize(r, c)
        self._update(nodes='—', cost='—', status='Ready', replans=0)
        self.t_time.set_text('Exec Time : —')
        self.draw_grid()

    def _cb_random(self, _):
        self.env.random_map(self.sl_density.val)
        self._update(status='Random map generated')
        self.draw_grid()

    def _cb_clear(self, _):
        self.env.clear()
        self._update(nodes='—', cost='—', status='Cleared', replans=0)
        self.t_time.set_text('Exec Time : —')
        self.draw_grid()

    def _cb_setstart(self, _):
        self.setting_start = True
        self.setting_goal  = False
        self._set_status('Click a cell to set Start', '#ffaa00')

    def _cb_setgoal(self, _):
        self.setting_goal  = True
        self.setting_start = False
        self._set_status('Click a cell to set Goal', '#ffaa00')

    def _cb_search(self, _):
        if self.is_running: return
        if self.dynamic_mode:
            self._run_dynamic()
        else:
            self._run_static()

    def _cb_dynamic(self, _):
        self.dynamic_mode = not self.dynamic_mode
        lbl = 'Dynamic: ON' if self.dynamic_mode else 'Dynamic: OFF'
        self.btn_dynamic.label.set_text(lbl)
        self.btn_dynamic.ax.set_facecolor('#006400' if self.dynamic_mode else '#0f3460')
        self.fig.canvas.draw_idle()

    def _cb_stop(self, _):
        self.is_running = False
        self._set_status('Stopped', '#ff4444')

    def _cb_click(self, event):
        if event.inaxes != self.ax or event.button != 1:
            return
        c = int(round(event.xdata))
        r = int(round(event.ydata))
        if not (0 <= r < self.env.rows and 0 <= c < self.env.cols):
            return

        if self.setting_start:
            if (r, c) != self.env.goal and self.env.grid[r, c] == 0:
                self.env.start = (r, c)
                self.setting_start = False
                self._update(status='Start node updated')
                self.draw_grid()
        elif self.setting_goal:
            if (r, c) != self.env.start and self.env.grid[r, c] == 0:
                self.env.goal = (r, c)
                self.setting_goal = False
                self._update(status='Goal node updated')
                self.draw_grid()
        else:
            self.env.toggle(r, c)
            self.draw_grid()


# ─────────────────────────────────────────────
#  Entry Point
# ─────────────────────────────────────────────
if __name__ == '__main__':
    plt.style.use('dark_background')
    PathfindingGUI()
