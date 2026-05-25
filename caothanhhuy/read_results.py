# -*- coding: utf-8 -*-
"""
read_results.py — Đọc file JSON kết quả và in ra chỉ số + đường đi
===================================================================
Cách dùng:
  python read_results.py                              # đọc tất cả file trong results/
  python read_results.py results/map_01_eval.json     # đọc 1 file cụ thể
  python read_results.py results/map_01_eval.json --run -1          # chỉ run cuối
  python read_results.py results/map_01_eval.json --run 0           # chỉ run đầu tiên
  python read_results.py results/map_01_eval.json --solver ACO      # chỉ 1 solver
  python read_results.py results/map_01_eval.json --no-path         # ẩn đường đi
  python read_results.py results/map_01_eval.json --full-path       # in toàn bộ path
"""

import sys, os, json, glob, argparse

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

_HERE = os.path.dirname(os.path.abspath(__file__))


# ──────────────────────────────────────────────────────────────────
# Helpers hiển thị
# ──────────────────────────────────────────────────────────────────
def _fmt(val, width=12):
    """Format một giá trị số hoặc None thành chuỗi căn phải."""
    if val is None:
        return f"{'N/A':>{width}}"
    if isinstance(val, float):
        return f"{val:>{width}.4f}"
    return f"{str(val):>{width}}"


def _print_metrics(solver_name, m, full_path=False, show_path=True):
    """In toàn bộ chỉ số và đường đi của một solver."""
    pad = "    "
    print(f"\n  ┌─ {solver_name} {'─'*(50-len(solver_name))}")

    if "error" in m:
        print(f"  │  ⚠ LỖI: {m['error']}")
        print("  └" + "─"*52)
        return

    metrics = [
        ("path_length",  "Độ dài đường đi",        "ô lưới", False),
        ("safety_mean",  "An toàn trung bình",      "(cao hơn = tốt)", False),
        ("safety_min",   "An toàn thấp nhất",       "(điểm nguy hiểm nhất)", False),
        ("risk",         "Rủi ro va chạm (tổng)",   "(thấp hơn = tốt)", False),
        ("risk_mean",    "Rủi ro trung bình/bước",  "", False),
        ("total_cost",   "Total cost",              "w1·L + w2·risk", False),
        ("F_score",      "F_score",                 "hàm mục tiêu gốc", False),
        ("time_fmm_sec", "Thời gian FMM",           "giây", False),
        ("time_tsp_sec", "Thời gian TSP solver",    "giây", False),
        ("time_total_sec","Thời gian tổng",         "giây", False),
    ]

    for key, label, unit, _ in metrics:
        val = m.get(key)
        if val is None:
            continue
        if isinstance(val, float):
            formatted = f"{val:.6f}"
        else:
            formatted = str(val)
        print(f"  │  {label:<30} = {formatted:>12}  {unit}")

    # Thứ tự thăm checkpoint
    route = m.get("route")
    if route:
        print(f"  │  {'Thứ tự checkpoint':<30}   {route}")

    cells_count = m.get("cells_count")
    if cells_count:
        print(f"  │  {'Số bước trên đường đi':<30} = {cells_count:>12}")

    # Đường đi dạng list of tuple
    if show_path:
        coords = m.get("path_coords")
        if coords:
            # Chuyển về list of tuple (JSON lưu dạng list of list)
            path_tuples = [tuple(p) for p in coords]
            print(f"  │")
            if full_path:
                print(f"  │  path_coords ({len(path_tuples)} bước) =")
                # In từng dòng 10 tuple một cho dễ đọc
                chunk = 10
                for i in range(0, len(path_tuples), chunk):
                    row_str = ", ".join(str(t) for t in path_tuples[i:i+chunk])
                    prefix = "  │      " if i > 0 else "  │      "
                    print(f"{prefix}[{row_str}{',' if i + chunk < len(path_tuples) else ''}]")
            else:
                preview = path_tuples[:10]
                rest    = len(path_tuples) - 10
                preview_str = "[" + ", ".join(str(t) for t in preview)
                if rest > 0:
                    preview_str += f", ... +{rest} bước]"
                else:
                    preview_str += "]"
                print(f"  │  path_coords ({len(path_tuples)} bước) = {preview_str}")
        else:
            print(f"  │  path_coords  = (không có — chạy lại evaluate.py để cập nhật)")

    print("  └" + "─"*52)


def _print_comparison(run):
    """In bảng so sánh nhanh các solver trong một run."""
    solvers = run.get("solvers", {})
    valid   = {k: v for k, v in solvers.items() if "error" not in v}
    if len(valid) < 2:
        return

    cols = ["path_length", "safety_mean", "risk", "total_cost", "F_score", "time_total_sec"]
    hdr  = f"  {'Solver':<22}" + "".join(f"{c:>13}" for c in cols)
    print("\n" + "  " + "═"*(len(hdr)-2))
    print("  SO SÁNH NHANH")
    print(hdr)
    print("  " + "─"*(len(hdr)-2))
    for name, m in valid.items():
        row = f"  {name:<22}"
        for c in cols:
            val = m.get(c)
            row += f"{_fmt(val, 13)}"
        print(row)
    print("  " + "═"*(len(hdr)-2))

    # Highlight bộ giải tốt nhất theo từng tiêu chí
    criteria = [
        ("path_length",   "Đường ngắn nhất",  True),
        ("safety_mean",   "An toàn nhất",      False),
        ("risk",          "Rủi ro thấp nhất",  True),
        ("total_cost",    "Chi phí thấp nhất", True),
    ]
    print()
    for key, label, lower_better in criteria:
        vals = {k: v[key] for k, v in valid.items()
                if key in v and v[key] is not None}
        if vals:
            best = (min if lower_better else max)(vals, key=vals.__getitem__)
            print(f"  ✓ {label:<24}: {best}  ({vals[best]})")


# ──────────────────────────────────────────────────────────────────
# Đọc và in một file JSON
# ──────────────────────────────────────────────────────────────────
def print_file(json_path, run_index=None, solver_filter=None,
               show_path=True, full_path=False):
    """
    Đọc một file JSON và in kết quả.

    run_index  : None = tất cả, 0 = run đầu, -1 = run cuối, v.v.
    solver_filter : None = tất cả solver, hoặc tên cụ thể (không phân biệt hoa/thường)
    show_path  : True = in path_coords
    full_path  : True = in toàn bộ path (không rút gọn)
    """
    if not os.path.isfile(json_path):
        print(f"[LỖI] Không tìm thấy file: {json_path}")
        return

    with open(json_path, "r", encoding="utf-8") as f:
        record = json.load(f)

    map_name = record.get("map", os.path.basename(json_path))
    runs     = record.get("runs", [])

    print("\n" + "╔" + "═"*60 + "╗")
    print(f"║  Bản đồ : {map_name:<49}║")
    print(f"║  File   : {json_path:<49}║")
    print(f"║  Số run : {len(runs):<49}║")
    print("╚" + "═"*60 + "╝")

    # Chọn run cần in
    if run_index is not None:
        try:
            runs_to_print = [(run_index % len(runs), runs[run_index])]
        except IndexError:
            print(f"[LỖI] run_index={run_index} vượt quá số run ({len(runs)}).")
            return
    else:
        runs_to_print = list(enumerate(runs))

    for idx, run in runs_to_print:
        ts  = run.get("timestamp", "?")
        cfg = run.get("config", {})
        print(f"\n{'━'*62}")
        print(f"  RUN #{idx}   |   {ts}")
        print(f"  w1={cfg.get('w1')}, w2={cfg.get('w2')}, "
              f"a_dgree={cfg.get('a_dgree')}, "
              f"T_max={cfg.get('T_max')}")
        print(f"  Checkpoints={cfg.get('checkpoints')}, "
              f"safe_medium={cfg.get('safe_medium')}, "
              f"perimeter={cfg.get('perimeter_map')}")
        print(f"{'━'*62}")

        solvers = run.get("solvers", {})
        for name, m in solvers.items():
            # Lọc solver nếu có
            if solver_filter and name.lower() != solver_filter.lower():
                continue
            _print_metrics(name, m, full_path=full_path, show_path=show_path)

        # Bảng so sánh (chỉ khi in nhiều solver)
        if solver_filter is None:
            _print_comparison(run)


# ──────────────────────────────────────────────────────────────────
# Lấy danh sách path_coords từ file JSON
# ──────────────────────────────────────────────────────────────────
def get_path_coords(json_path, run_index=-1, solver="Christofides"):
    """
    Trả về path_coords dạng list of tuple từ file JSON.
    Dùng để import trong notebook hoặc code khác.

    Ví dụ:
        from read_results import get_path_coords
        path = get_path_coords("results/map_01_eval.json", solver="ACO")
        # path = [(51,133), (50,133), ...]
    """
    with open(json_path, "r", encoding="utf-8") as f:
        record = json.load(f)
    runs = record.get("runs", [])
    if not runs:
        return []
    run = runs[run_index]
    coords = run.get("solvers", {}).get(solver, {}).get("path_coords", [])
    return [tuple(p) for p in coords]


# ──────────────────────────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Đọc kết quả evaluate.py từ file JSON",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument(
        "files", nargs="*",
        help="Đường dẫn file(s) JSON.\n"
             "Mặc định: tất cả file *_eval.json trong results/",
    )
    parser.add_argument(
        "--run", type=int, default=None, metavar="N",
        help="Chỉ in run thứ N (0-based, -1 = cuối cùng).\n"
             "Mặc định: in tất cả run.",
    )
    parser.add_argument(
        "--solver", default=None, metavar="TÊN",
        help="Chỉ in kết quả của một solver.\n"
             "Ví dụ: --solver ACO  hoặc  --solver Christofides",
    )
    parser.add_argument(
        "--no-path", dest="show_path", action="store_false",
        help="Ẩn phần đường đi (path_coords).",
    )
    parser.add_argument(
        "--full-path", dest="full_path", action="store_true",
        help="In toàn bộ path_coords, không rút gọn.",
    )
    args = parser.parse_args()

    # Tìm file cần đọc
    if args.files:
        json_files = []
        for f in args.files:
            if not os.path.isabs(f):
                f = os.path.join(_HERE, f)
            json_files.append(f)
    else:
        json_files = sorted(
            glob.glob(os.path.join(_HERE, "results", "*_eval.json"))
        )
        if not json_files:
            print("Không tìm thấy file *_eval.json nào trong results/")
            print("Hãy chạy evaluate.py trước để tạo kết quả.")
            sys.exit(0)

    for jf in json_files:
        print_file(
            jf,
            run_index   = args.run,
            solver_filter = args.solver,
            show_path   = args.show_path,
            full_path   = args.full_path,
        )
