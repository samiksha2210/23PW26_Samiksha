import math
import json
import os
import time
import random

EPS = 1e-9
OFFSET = 1e-3  # small buffer to ensure waypoints lie strictly outside zones


def kinematic_time(distance, vmax, acc):
    if distance <= EPS:
        return 0.0
    d_crit = (vmax * vmax) / acc
    if distance >= d_crit:
        return (distance / vmax) + (vmax / acc)
    else:
        return 2.0 * math.sqrt(distance / acc)

def rotation_time(a1, a2, v_cam, a_cam):
    diff = (a2 - a1) % 360
    diff = abs(diff)
    eff = diff % 90
    rot = min(eff, 90 - eff)
    return kinematic_time(rot, v_cam, a_cam)


def normalize_rect(x1, y1, x2, y2):
    xmin = min(x1, x2); xmax = max(x1, x2)
    ymin = min(y1, y2); ymax = max(y1, y2)
    return (xmin, ymin, xmax, ymax)

def point_in_rect(p, rect):
    x, y = p
    xmin, ymin, xmax, ymax = rect
    return (xmin - EPS) <= x <= (xmax + EPS) and (ymin - EPS) <= y <= (ymax + EPS)

def on_segment(a, b, c):
    return (min(a[0], b[0]) - EPS <= c[0] <= max(a[0], b[0]) + EPS) and \
           (min(a[1], b[1]) - EPS <= c[1] <= max(a[1], b[1]) + EPS)

def orient(a, b, c):
    return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])

def segments_intersect(p1, p2, q1, q2):
    o1 = orient(p1, p2, q1)
    o2 = orient(p1, p2, q2)
    o3 = orient(q1, q2, p1)
    o4 = orient(q1, q2, p2)

    if abs(o1) < EPS and on_segment(p1, p2, q1): return True
    if abs(o2) < EPS and on_segment(p1, p2, q2): return True
    if abs(o3) < EPS and on_segment(q1, q2, p1): return True
    if abs(o4) < EPS and on_segment(q1, q2, p2): return True

    return (o1 > 0) != (o2 > 0) and (o3 > 0) != (o4 > 0)

def segment_intersects_rect(p1, p2, rect):
    xmin, ymin, xmax, ymax = rect
    if point_in_rect(p1, rect) or point_in_rect(p2, rect):
        return True
    bl = (xmin, ymin); br = (xmax, ymin); tl = (xmin, ymax); tr = (xmax, ymax)
    edges = [(bl, br), (br, tr), (tr, tl), (tl, bl)]
    for e1, e2 in edges:
        if segments_intersect(p1, p2, e1, e2):
            return True
    mid = ((p1[0]+p2[0])/2.0, (p1[1]+p2[1])/2.0)
    if point_in_rect(mid, rect):
        return True
    return False

def segment_blocked(p1, p2, zones):
    for rect in zones:
        if segment_intersects_rect(p1, p2, rect):
            return True
    return False



def zone_waypoints(rect):
    xmin, ymin, xmax, ymax = rect
    pts = [
        (xmin - OFFSET, ymin - OFFSET),
        (xmin - OFFSET, ymax + OFFSET),
        (xmax + OFFSET, ymin - OFFSET),
        (xmax + OFFSET, ymax + OFFSET),
    ]
    pts += [
        (xmin - OFFSET, (ymin + ymax)/2.0),  # left side
        (xmax + OFFSET, (ymin + ymax)/2.0),  # right side
        ((xmin + xmax)/2.0, ymin - OFFSET),  # bottom side
        ((xmin + xmax)/2.0, ymax + OFFSET),  # top side
    ]
    return pts

def build_visibility_nodes(p_start, p_goal, zones):
    nodes = [tuple(p_start), tuple(p_goal)]
    for rect in zones:
        nodes.extend(zone_waypoints(rect))
    return nodes

def edge_visible(a, b, zones):
    return not segment_blocked(a, b, zones)

def dijkstra_visibility(p_start, p_goal, zones):
    nodes = build_visibility_nodes(p_start, p_goal, zones)
    n = len(nodes)
    adj = [[] for _ in range(n)]
    for i in range(n):
        for j in range(i+1, n):
            a = nodes[i]; b = nodes[j]
            if edge_visible(a, b, zones):
                w = math.hypot(b[0]-a[0], b[1]-a[1])
                adj[i].append((j, w))
                adj[j].append((i, w))

    import heapq
    start_idx = 0
    goal_idx = 1
    dist = [float('inf')] * n
    parent = [-1] * n
    dist[start_idx] = 0.0
    heap = [(0.0, start_idx)]
    while heap:
        cd, u = heapq.heappop(heap)
        if cd > dist[u] + EPS:
            continue
        if u == goal_idx:
            break
        for v, w in adj[u]:
            nd = cd + w
            if nd + EPS < dist[v]:
                dist[v] = nd
                parent[v] = u
                heapq.heappush(heap, (nd, v))

    if dist[goal_idx] == float('inf'):
        return [p_start, p_goal]

    path = []
    cur = goal_idx
    while cur != -1:
        path.append(nodes[cur])
        cur = parent[cur]
    path.reverse()
    return path



def is_same_direction(v1, v2):
    cross = v1[0]*v2[1] - v1[1]*v2[0]
    dot = v1[0]*v2[0] + v1[1]*v2[1]
    return abs(cross) <= 1e-9 and dot > 0

def compress_polyline(points):
    if len(points) <= 2:
        return points[:]
    out = [points[0]]
    prev = points[0]
    cur = points[1]
    for nxt in points[2:]:
        v1 = (cur[0]-prev[0], cur[1]-prev[1])
        v2 = (nxt[0]-cur[0], nxt[1]-cur[1])
        if is_same_direction(v1, v2):
            # merge cur
            cur = nxt
        else:
            out.append(cur)
            prev = cur
            cur = nxt
    out.append(cur)
    return out



def translation_time_for_path(points, v_stage, a_stage):
    pts = compress_polyline(points)
    t = 0.0
    for i in range(len(pts)-1):
        d = math.hypot(pts[i+1][0]-pts[i][0], pts[i+1][1]-pts[i][1])
        t += kinematic_time(d, v_stage, a_stage)
    return t, pts

def compute_move(p1, a1, p2, a2, v_stage, a_stage, v_cam, a_cam, zones):
    if not segment_blocked(p1, p2, zones):
        way = [p1, p2]
    else:
        way = dijkstra_visibility(p1, p2, zones)
        safe = True
        for i in range(len(way)-1):
            if segment_blocked(way[i], way[i+1], zones):
                safe = False
                break
        if not safe:
            nodes = build_visibility_nodes(p1, p2, zones)
            chain = [p1]
            cur = p1
            max_iters = 5 * len(nodes)
            iters = 0
            while segment_blocked(cur, p2, zones) and iters < max_iters:
                iters += 1
                candidates = [w for w in nodes if not segment_blocked(cur, w, zones)]
                if not candidates:
                    break
                candidates.sort(key=lambda w: (math.hypot(w[0]-cur[0], w[1]-cur[1]) +
                                               0.5*math.hypot(p2[0]-w[0], p2[1]-w[1])))
                nxt = candidates[0]
                if len(chain) >= 2 and nxt == chain[-2]:
                    break
                chain.append(nxt)
                cur = nxt
            chain.append(p2)
            way = chain
        way = [pt for pt in way if not any(point_in_rect(pt, z) for z in zones)]
        if way[0] != p1: way.insert(0, p1)
        if way[-1] != p2: way.append(p2)

    t_xy, way_compressed = translation_time_for_path(way, v_stage, a_stage)
    t_rot = rotation_time(a1, a2, v_cam, a_cam)
    return max(t_xy, t_rot), way_compressed



def solve_tsp_dp(dist):
    n = len(dist)
    INF = float('inf')
    dp = [[INF]*n for _ in range(1<<n)]
    parent = [[-1]*n for _ in range(1<<n)]
    dp[1][0] = 0.0  # start at node 0
    for mask in range(1<<n):
        for u in range(n):
            if not (mask & (1<<u)): 
                continue
            if dp[mask][u] == INF:
                continue
            for v in range(1, n):
                if (mask & (1<<v)) == 0:
                    nm = mask | (1<<v)
                    nv = dp[mask][u] + dist[u][v]
                    if nv + EPS < dp[nm][v]:
                        dp[nm][v] = nv
                        parent[nm][v] = u
    full = (1<<n) - 1
    end = min(range(1, n), key=lambda i: dp[full][i])
    path = []
    cur = end
    mask = full
    while cur != -1:
        path.append(cur)
        p = parent[mask][cur]
        mask ^= (1<<cur)
        cur = p
    path.reverse()
    if path[0] != 0:
        path.insert(0, 0)
    return path

def two_opt(path, dist):
    n = len(path)
    improved = True
    while improved:
        improved = False
        for i in range(1, n-1):
            for j in range(i+1, n):
                a, b = path[i-1], path[i]
                c, d = path[j-1], path[j] if j < n else None
                old = dist[a][b] + (dist[c][d] if d is not None else 0)
                new = dist[a][c] + (dist[b][d] if d is not None else 0)
                if new + EPS < old:
                    path[i:j] = reversed(path[i:j])
                    improved = True
    return path

def solve_vns(dist, time_budget=2.8):
    n = len(dist)
    best_cost = float('inf')
    best_path = []
    start = time.time()
    while time.time() - start < time_budget:
        visited = [False]*n
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
        cost = sum(dist[path[i]][path[i+1]] for i in range(n-1))
        if cost + EPS < best_cost:
            best_cost = cost
            best_path = path[:]
    if not best_path:
        best_path = list(range(n))
    return best_path

# -------------------------------
# Utilities
# -------------------------------

def die_center_and_angle(corners):
    cx = sum(p[0] for p in corners)/4.0
    cy = sum(p[1] for p in corners)/4.0
    dx = corners[1][0] - corners[0][0]
    dy = corners[1][1] - corners[0][1]
    ang = (math.degrees(math.atan2(dy, dx)) + 360) % 360
    return (cx, cy, ang)


def solve(input_file, output_file):
    with open(input_file) as f:
        data = json.load(f)

    init_pos = tuple(data["InitialPosition"])
    init_angle = data["InitialAngle"] % 360
    v_stage = data["StageVelocity"]
    a_stage = data["StageAcceleration"]
    v_cam = data["CameraVelocity"]
    a_cam = data["CameraAcceleration"]

    zones = []
    for z in data.get("ForbiddenZones", []):
        x1, y1 = z["BottomLeft"]
        x2, y2 = z["TopRight"]
        zones.append(normalize_rect(x1, y1, x2, y2))

    dies = data["Dies"]
    centers = [(init_pos[0], init_pos[1], init_angle)]
    for d in dies:
        centers.append(die_center_and_angle(d["Corners"]))

    n = len(centers)

    # Pairwise leg cost and path
    dist = [[0.0]*n for _ in range(n)]
    leg_path = [[None]*n for _ in range(n)]

    for i in range(n):
        for j in range(n):
            if i == j:
                dist[i][j] = 0.0
                leg_path[i][j] = [(centers[i][0], centers[i][1])]
            else:
                p1 = (centers[i][0], centers[i][1])
                a1 = centers[i][2]
                p2 = (centers[j][0], centers[j][1])
                a2 = centers[j][2]
                t, path_pts = compute_move(p1, a1, p2, a2, v_stage, a_stage, v_cam, a_cam, zones)
                # Final validation: ensure every segment avoids zones
                valid = True
                for k in range(len(path_pts)-1):
                    if segment_blocked(path_pts[k], path_pts[k+1], zones):
                        valid = False
                        break
                if not valid:
                    t = 1e9
                    path_pts = [p1, p2]
                dist[i][j] = t
                leg_path[i][j] = path_pts

    if n <= 19:
        order = solve_tsp_dp(dist)
    else:
        order = solve_vns(dist, time_budget=2.8)
        if order[0] != 0:
            order.remove(0)
            order.insert(0, 0)

    total_time = 0.0
    full_points = [[centers[order[0]][0], centers[order[0]][1]]]
    for k in range(len(order)-1):
        i = order[k]
        j = order[k+1]
        t_leg = dist[i][j]
        total_time += t_leg
        seg = leg_path[i][j]
        for pt in seg[1:]:
            full_points.append([pt[0], pt[1]])

    for k in range(len(full_points)-1):
        a = tuple(full_points[k])
        b = tuple(full_points[k+1])
        if segment_blocked(a, b, zones):
            total_time = 1e9
            break

    out = {
        "TotalTime": total_time,
        "Path": full_points
    }
    with open(output_file, "w") as f:
        json.dump(out, f, indent=2)

    print(f"{input_file} solved → Time = {total_time:.4f}")


if __name__ == "__main__":
    input_files = [
        "Input_Milestone4_Testcase1.json",
        "Input_Milestone4_Testcase2.json",
        "Input_Milestone4_Testcase3.json",
        "Input_Milestone4_Testcase4.json"
    ]
    output_files = [
        "TestCase_4_1.json",
        "TestCase_4_2.json",
        "TestCase_4_3.json",
        "TestCase_4_4.json"
    ]

    for inp, out in zip(input_files, output_files):
        if os.path.exists(inp):
            solve(inp, out)
