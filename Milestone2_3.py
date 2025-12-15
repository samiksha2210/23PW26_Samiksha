import json
import math

with open("Input_Milestone2_Testcase4.json", "r") as f:
    data = json.load(f)

stage_velocity = data['StageVelocity']
camera_velocity = data['CameraVelocity']
initial_pos = data['InitialPosition']
initial_angle = data['InitialAngle']
dies = data['Dies']

centers = [initial_pos]
die_angles = [initial_angle]

def min_angular_diff(a1, a2):
    diff = abs(a1 - a2) % 360
    return min(diff, 360 - diff)

def get_die_angle(corners):
    angles = []
    for i in range(4):
        dx = corners[(i+1)%4][0] - corners[i][0]
        dy = corners[(i+1)%4][1] - corners[i][1]
        angle = math.degrees(math.atan2(dy, dx))
        if angle < 0:
            angle += 360
        angles.append(angle)
    return min(angles)

for die in dies:
    corners = die['Corners']
    cx = sum(c[0] for c in corners) / 4
    cy = sum(c[1] for c in corners) / 4
    centers.append([cx, cy])
    die_angles.append(get_die_angle(corners))

n = len(centers)  
def edge_cost(i, j):
    trans_dist = math.dist(centers[i], centers[j])
    trans_time = trans_dist / stage_velocity
    rot_diff = min_angular_diff(die_angles[i], die_angles[j])
    rot_time = rot_diff / camera_velocity
    return max(trans_time, rot_time)

dist = [[0.0 for _ in range(n)] for _ in range(n)]
for i in range(n):
    for j in range(n):
        if i != j:
            dist[i][j] = edge_cost(i, j)

INF = float('inf')

dp = [[INF] * n for _ in range(1 << n)]
dp[1][0] = 0.0 

pred = [[-1] * n for _ in range(1 << n)]

for mask in range(1 << n):
    for u in range(n):
        if dp[mask][u] == INF:
            continue
        for v in range(n):
            if mask & (1 << v):
                continue  
            new_mask = mask | (1 << v)
            new_cost = dp[mask][u] + dist[u][v]
            if new_cost < dp[new_mask][v]:
                dp[new_mask][v] = new_cost
                pred[new_mask][v] = u

full_mask = (1 << n) - 1
min_path_cost = min(dp[full_mask][k] for k in range(n) if dp[full_mask][k] < INF)

best_path = None
best_time = min_path_cost

for possible_end in range(n):
    if dp[full_mask][possible_end] != min_path_cost:
        continue
    
    path = []
    current_mask = full_mask
    current = possible_end
    while current != -1:
        path.append(current)
        prev = pred[current_mask][current]
        if prev == -1:
            break
        current_mask ^= (1 << current) 
        current = prev
    path.reverse()
    
    if best_path is None or path < best_path:
        best_path = path


print(best_path)  
best_time = min_path_cost
final_path_coords = [centers[i] for i in best_path]

ans = {
    "TotalTime": round(best_time, 3),
    "Path": final_path_coords
}

with open("TestCase_2_44.json", "w") as f:
    json.dump(ans, f, indent=4)

