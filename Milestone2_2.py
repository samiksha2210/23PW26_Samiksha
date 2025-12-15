import json
import math
import random

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
    return min(angles, key=lambda a: min_angular_diff(a, initial_angle))

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

def total_path_cost(p):
    return sum(edge_cost(p[k], p[k+1]) for k in range(len(p)-1))

def two_opt_improve(path):
    current_path = path[:]
    current_time = total_path_cost(current_path)
    improved = True
    while improved:
        improved = False
        for i in range(1, n-1):
            for j in range(i+2, n):
                new_path = current_path[:i] + current_path[i:j+1][::-1] + current_path[j+1:]
                new_time = total_path_cost(new_path)
                if new_time < current_time:
                    current_path = new_path
                    current_time = new_time
                    improved = True
                    break
            if improved:
                break
    return current_path, current_time

path = [0]
visited = set([0])
current = 0
while len(path) < n:
    next_node = min((c for c in range(n) if c not in visited), key=lambda c: edge_cost(current, c))
    path.append(next_node)
    visited.add(next_node)
    current = next_node

best_path, best_time = two_opt_improve(path)

for _ in range(7000): 
    remaining = list(range(1, n))
    random.shuffle(remaining)
    random_path = [0] + remaining
    improved_path, improved_time = two_opt_improve(random_path)
    if improved_time < best_time:
        best_time = improved_time
        best_path = improved_path

print(f"Most accurate Total Time found: {best_time:.3f} seconds")

final_path_coords = [centers[i] for i in best_path]
ans = {
    "TotalTime": round(best_time, 3),
    "Path": final_path_coords
}

with open("TestCase_2_4.json", "w") as f:
    json.dump(ans, f, indent=4)

