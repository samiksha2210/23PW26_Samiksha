import math
import json
import random
import time
import sys

sys.setrecursionlimit(5000)

def kinematic_time(distance, vmax, acc):
    if distance <= 1e-9:
        return 0.0
    d_crit = (vmax * vmax) / acc
    if distance >= d_crit:
        return (distance / vmax) + (vmax / acc)
    else:
        return 2.0 * math.sqrt(distance / acc)
    
def move_cost(p1, p2, v_stage, a_stage, v_cam, a_cam):
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    dist_xy = math.hypot(dx, dy)
    t_xy = kinematic_time(dist_xy, v_stage, a_stage)

    diff = abs(p2[2] - p1[2])
    eff = diff % 90
    rot = min(eff, 90 - eff)
    t_rot = kinematic_time(rot, v_cam, a_cam)

    return max(t_xy, t_rot)

def solve_dp(dist):
    n = len(dist)
    INF = float("inf")
    dp = [[INF] * n for _ in range(1 << n)]
    parent = [[-1] * n for _ in range(1 << n)]
    dp[1][0] = 0.0

    for mask in range(1 << n):
        for u in range(n):
            if dp[mask][u] == INF:
                continue
            for v in range(1, n):
                if not (mask & (1 << v)):
                    nm = mask | (1 << v)
                    val = dp[mask][u] + dist[u][v]
                    if val < dp[nm][v]:
                        dp[nm][v] = val
                        parent[nm][v] = u

    full = (1 << n) - 1
    end = min(range(1, n), key=lambda i: dp[full][i])
    cost = dp[full][end]

    path = []
    mask = full
    cur = end
    while cur != -1:
        path.append(cur)
        p = parent[mask][cur]
        mask ^= (1 << cur)
        cur = p

    return cost, path[::-1]

def two_opt(path, dist):
    n = len(path)
    improved = True

    while improved:
        improved = False
        for i in range(1, n - 1):
            for j in range(i + 1, n):
                a, b = path[i - 1], path[i]
                c, d = path[j - 1], path[j] if j < n else None

                old = dist[a][b] + (dist[c][d] if d is not None else 0)
                new = dist[a][c] + (dist[b][d] if d is not None else 0)

                if new < old - 1e-9:
                    path[i:j] = reversed(path[i:j])
                    improved = True
    return path

def solve_vns(dist, time_budget=2.8):
    n = len(dist)
    best_cost = float("inf")
    best_path = []

    start_time = time.time()

    while time.time() - start_time < time_budget:
        visited = [False] * n
        path = [0]
        visited[0] = True
        cur = 0

        while len(path) < n:
            cand = [i for i in range(n) if not visited[i]]
            cand.sort(key=lambda x: dist[cur][x])
            nxt = random.choice(cand[:min(3, len(cand))])
            path.append(nxt)
            visited[nxt] = True
            cur = nxt

        path = two_opt(path, dist)

        cost = sum(dist[path[i]][path[i + 1]] for i in range(n - 1))
        if cost < best_cost:
            best_cost = cost
            best_path = path[:]

    return best_cost, best_path

def solve(input_file, output_file):
    with open(input_file) as f:
        data = json.load(f)

    init_pos = data["InitialPosition"]
    init_angle = data["InitialAngle"] % 360

    v_stage = data["StageVelocity"]
    a_stage = data["StageAcceleration"]
    v_cam = data["CameraVelocity"]
    a_cam = data["CameraAcceleration"]

    dies = data["Dies"]

    centers = [(init_pos[0], init_pos[1], init_angle)]

    for d in dies:
        c = d["Corners"]
        cx = sum(p[0] for p in c) / 4
        cy = sum(p[1] for p in c) / 4
        dx = c[1][0] - c[0][0]
        dy = c[1][1] - c[0][1]
        ang = (math.degrees(math.atan2(dy, dx)) + 360) % 360
        centers.append((cx, cy, ang))

    n = len(centers)

    dist = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            if i != j:
                dist[i][j] = move_cost(
                    centers[i], centers[j],
                    v_stage, a_stage, v_cam, a_cam
                )

    if n <= 19:
        total, path = solve_dp(dist)
    else:
        total, path = solve_vns(dist)

    if path[0] != 0:
        path.remove(0)
        path.insert(0, 0)

    final_time = sum(dist[path[i]][path[i + 1]] for i in range(n - 1))

    output_path = [data["InitialPosition"]]
    for idx in path[1:]:
        cx, cy, _ = centers[idx]
        output_path.append([cx, cy])

    with open(output_file, "w") as f:
        json.dump({
            "TotalTime": final_time,
            "Path": output_path
        }, f, indent=4)

    print(f"{input_file} solved → Time = {final_time:.4f}")

solve("Input_Milestone3_Testcase1.json", "TestCase3_1.json")
solve("Input_Milestone3_Testcase2.json", "TestCase3_2.json")
solve("Input_Milestone3_Testcase3.json", "TestCase3_3.json")
solve("Input_Milestone3_Testcase4.json", "TestCase3_4.json")