"""
Patch visualization.ipynb:
  1. Fix cell[1]: wrap FMF JSON read in try/except -> path_from_fmf = None when not found
  2. Replace cell[7]: use path_from_fmf from cell[1] + computed_metrics from cell[4]
"""
import json, re
from pathlib import Path

NB_PATH = "visualization.ipynb"
nb = json.load(open(NB_PATH, encoding="utf-8"))

# ─────────────────────────────────────────────────────────────────────────────
# 1. Fix cell[1]: wrap FMF section with try/except
# ─────────────────────────────────────────────────────────────────────────────
old_c1 = "".join(nb["cells"][1]["source"])

# Find the FMF block to replace (from JSON_FILE_FMF definition to path_from_fmf assignment)
OLD_FMF_BLOCK = """\
# Đọc path từ file JSON của FMF_new
JSON_FILE_FMF = "FMF_new/results/mixed200.json"

with open(JSON_FILE_FMF) as f:
    data_fmf = json.load(f)

path_from_fmf = [tuple(p) for p in data_fmf["runs"]["path"]]"""

NEW_FMF_BLOCK = """\
# Đọc path từ file JSON của FMF_new
JSON_FILE_FMF = "FMF_new/results/mixed200.json"

try:
    with open(JSON_FILE_FMF) as f:
        data_fmf = json.load(f)
    path_from_fmf = [tuple(p) for p in data_fmf["runs"]["path"]]
    print(f"FMF path loaded: {len(path_from_fmf)} steps")
except FileNotFoundError:
    data_fmf = {}
    path_from_fmf = None
    print(f"[INFO] FMF file not found: {JSON_FILE_FMF} -- bo qua FMF")"""

if OLD_FMF_BLOCK in old_c1:
    new_c1 = old_c1.replace(OLD_FMF_BLOCK, NEW_FMF_BLOCK)
    nb["cells"][1]["source"] = [new_c1]
    print("Patched cell[1]: FMF read wrapped in try/except")
elif "path_from_fmf = None" in old_c1:
    print("cell[1] already has try/except, skip")
else:
    print("WARNING: could not find FMF block in cell[1], manual check needed")
    print("Current cell[1] snippet around FMF:")
    idx = old_c1.find("FMF")
    print(repr(old_c1[max(0,idx-20):idx+200]))

# ─────────────────────────────────────────────────────────────────────────────
# 2. Replace cell[7]: FMF comparison using path_from_fmf + computed_metrics
# ─────────────────────────────────────────────────────────────────────────────
FMF_CELL_CODE = '''\
# ─── FMF vs ACO-GA-A*: so sanh metrics + ve ban do ─────────────────────────
# Yeu cau: da chay cell[1] (path_from_fmf) va cell[4] (grid_map, computed_metrics)

if path_from_fmf is None:
    print("[SKIP] FMF path chua co. Chay FMF truoc de sinh:", JSON_FILE_FMF)
else:
    # ── Tinh metrics FMF qua src/costs ──────────────────────────────────────
    computed_fmf = path_cost_components(grid_map, path_from_fmf, weights)

    # ── Lay metrics FMF tu JSON (neu co full_path_components) ───────────────
    fmf_json_metrics = data_fmf.get("full_path_components",
                       data_fmf.get("cost_components",
                       data_fmf.get("metrics", {})))
    if isinstance(fmf_json_metrics, dict) and fmf_json_metrics:
        fmf_ref = fmf_json_metrics
        fmf_src = "JSON"
    else:
        fmf_ref = computed_fmf
        fmf_src = "Computed"

    # ── Bang so sanh ─────────────────────────────────────────────────────────
    _K   = ["length", "risk", "energy", "collision_risk"]
    _LBL = {"length":"Length (m)", "risk":"Risk (Sv.h)",
            "energy":"Energy (turns)", "collision_risk":"Coll. Risk",
            "total":"Total Cost"}
    _W   = {"length":weights.omega_length, "risk":weights.omega_risk,
            "energy":weights.omega_energy,  "collision_risk":weights.omega_collision_risk}

    print()
    print("=" * 80)
    print(f"  COMPARISON:  ACO-GA-A*  vs  FMF [{fmf_src}]")
    print("=" * 80)
    print(f"  {'Metric':<20} {'w':>5}  {'ACO-GA-A*':>14}  {'FMF':>14}  {'Delta':>10}")
    print("-" * 80)
    for _k in _K:
        _a = computed_metrics[_k]
        _b = fmf_ref.get(_k, computed_fmf[_k])
        _d = _b - _a
        print(f"  {_LBL[_k]:<20} {_W[_k]:>5.2f}  {_a:>14.4f}  {_b:>14.4f}  {_d:>+10.4f}")
    print("-" * 80)
    _ta = computed_metrics["total"]
    _tb = fmf_ref.get("total", computed_fmf["total"])
    print(f"  {_LBL[\'total\']:<20} {\'\'!s:>5}  {_ta:>14.4f}  {_tb:>14.4f}  {_tb-_ta:>+10.4f}")
    print("=" * 80)
    print(f"  Steps : ACO-GA-A* = {len(PATH)}   FMF = {len(path_from_fmf)}")
    winner = "FMF" if _tb < _ta else ("ACO-GA-A*" if _ta < _tb else "Tie")
    print(f"  Winner (lower total): {winner}")

    # ── Ve ban do so sanh ────────────────────────────────────────────────────
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    from matplotlib.colors import ListedColormap
    import numpy as np

    _obs = grid_map.obstacle_grid
    _rad = grid_map.radiation_grid
    _nr, _nc = _obs.shape

    _base = np.ones((_nr, _nc), dtype=int)
    _base[_rad >= 0.5]              = 2
    _base[_rad >= grid_map.ri_max]  = 3
    _base[_obs == 1]                = 0

    _cmap = ListedColormap(["#222222","#87CEEB","#FFFF66","#FFB6B6"])
    _fig, _ax = plt.subplots(figsize=(max(12,_nc*0.28), max(9,_nr*0.28)))
    _ax.imshow(_base, cmap=_cmap, vmin=0, vmax=3, origin="upper", interpolation="nearest")

    # Ve ca hai duong di
    if PATH:
        _xs=[p[1] for p in PATH]; _ys=[p[0] for p in PATH]
        _ax.plot(_xs, _ys, color="red",  linewidth=1.8, zorder=4, label="ACO-GA-A*", alpha=0.80)
        _ax.scatter(_xs[0], _ys[0],  s=180, c="lime",   edgecolors="k", lw=1.2, zorder=7, marker="o")
        _ax.scatter(_xs[-1],_ys[-1], s=180, c="orange", edgecolors="k", lw=1.2, zorder=7, marker="o")

    _xf=[p[1] for p in path_from_fmf]; _yf=[p[0] for p in path_from_fmf]
    _ax.plot(_xf, _yf, color="cyan", linewidth=1.8, zorder=5, label="FMF", alpha=0.80)
    _ax.scatter(_xf[0], _yf[0],  s=180, c="lime",   edgecolors="k", lw=1.2, zorder=8, marker="D")
    _ax.scatter(_xf[-1],_yf[-1], s=180, c="yellow", edgecolors="k", lw=1.2, zorder=8, marker="D")

    for _idx,(_r,_c) in enumerate(np.argwhere(_obs==2), start=1):
        _ax.scatter(_c,_r, marker="*", s=350, c="blue", edgecolors="blue", lw=0.6, zorder=9)
        _ax.text(_c+0.3,_r+0.3,str(_idx), color="blue", fontsize=7, fontweight="bold",
                 ha="left", va="center", zorder=10)

    if SHOW_GRID_LINES:
        _ax.set_xticks(np.arange(-0.5,_nc,1), minor=True)
        _ax.set_yticks(np.arange(-0.5,_nr,1), minor=True)
        _ax.grid(which="minor", color="black", linestyle="-", linewidth=0.4)
        _ax.tick_params(which="both", bottom=False, left=False,
                        labelbottom=False, labelleft=False)
    _ax.set_xlim(-0.5,_nc-0.5); _ax.set_ylim(_nr-0.5,-0.5); _ax.set_aspect("equal")

    # Text-box so sanh
    _bx = "\\n".join([
        f"ACO-GA-A*  |  FMF ({fmf_src})",
        f"Steps: {len(PATH):5d}  |  {len(path_from_fmf):5d}",
        f"Length:{computed_metrics[\'length\']:9.2f}  |  {fmf_ref.get(\'length\',computed_fmf[\'length\']):9.2f}",
        f"Risk  :{computed_metrics[\'risk\']:9.4f}  |  {fmf_ref.get(\'risk\',computed_fmf[\'risk\']):9.4f}",
        f"Energy:{computed_metrics[\'energy\']:9.2f}  |  {fmf_ref.get(\'energy\',computed_fmf[\'energy\']):9.2f}",
        f"CollR :{computed_metrics[\'collision_risk\']:9.4f}  |  {fmf_ref.get(\'collision_risk\',computed_fmf[\'collision_risk\']):9.4f}",
        f"Total :{_ta:9.4f}  |  {_tb:9.4f}",
        f"Winner: {winner}",
    ])
    _ax.text(0.99,0.01,_bx, transform=_ax.transAxes, fontsize=8, family="monospace",
             va="bottom", ha="right",
             bbox=dict(boxstyle="round,pad=0.4",facecolor="white",alpha=0.92,
                       edgecolor="#333333",linewidth=1.2), zorder=11)

    _li = [
        mpatches.Patch(color="#222222", label="Obstacle"),
        mpatches.Patch(color="#87CEEB", label="Low risk"),
        mpatches.Patch(color="#FFFF66", label="Med risk"),
        mpatches.Patch(color="#FFB6B6", label="High risk"),
        plt.Line2D([0],[0], color="red",  lw=2, label="ACO-GA-A*"),
        plt.Line2D([0],[0], color="cyan", lw=2, label="FMF"),
        plt.Line2D([0],[0], marker="o", color="lime",   ls="None", ms=8, label="Start ACO"),
        plt.Line2D([0],[0], marker="D", color="lime",   ls="None", ms=8, label="Start FMF"),
        plt.Line2D([0],[0], marker="o", color="orange", ls="None", ms=8, label="End ACO"),
        plt.Line2D([0],[0], marker="D", color="yellow", ls="None", ms=8, label="End FMF"),
    ]
    _ax.legend(handles=_li, loc="upper right", fontsize=7, framealpha=0.85)
    _fig.suptitle(
        f"ACO-GA-A* (total={_ta:.4f})  vs  FMF (total={_tb:.4f})  |  map {_nr}x{_nc}",
        fontsize=10
    )
    _fig.tight_layout()
    if SAVE_PATH:
        from pathlib import Path as _P
        _sp = _P(SAVE_PATH); _sp.parent.mkdir(parents=True, exist_ok=True)
        _fig.savefig(_sp.with_stem(_sp.stem+"_fmf_vs_aco"), dpi=150, bbox_inches="tight")
    plt.show()
'''

# Replace or insert cell[7]
new_cell = {
    "cell_type": "code",
    "execution_count": None,
    "metadata": {},
    "outputs": [],
    "source": [FMF_CELL_CODE],
}

if len(nb["cells"]) > 7:
    nb["cells"][7] = new_cell
    print("Replaced cell[7] with corrected FMF comparison cell")
else:
    nb["cells"].append(new_cell)
    print(f"Appended FMF cell as cell[{len(nb['cells'])-1}]")

# ─────────────────────────────────────────────────────────────────────────────
# Save
# ─────────────────────────────────────────────────────────────────────────────
with open(NB_PATH, "w", encoding="utf-8") as f:
    json.dump(nb, f, ensure_ascii=False, indent=1)
print(f"Saved. Total cells: {len(nb['cells'])}")
