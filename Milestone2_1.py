from itertools import permutations
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
    
    best_angle = min(angles, key=lambda a: min_angular_diff(a, initial_angle))
    return best_angle

for die in dies:
    corners = die['Corners']
    cx = sum(c[0] for c in corners) / 4
    cy = sum(c[1] for c in corners) / 4
    centers.append([cx, cy])
    
    angle = get_die_angle(corners)
    die_angles.append(angle)

def path_total_time(path_indices):
    total_time = 0.0
    current_angle = initial_angle
    
    for i in range(len(path_indices) - 1):
        from_idx = path_indices[i]
        to_idx = path_indices[i + 1]
        
        trans_dist = math.dist(centers[from_idx], centers[to_idx])
        trans_time = trans_dist / stage_velocity
        
       
        rot_diff = min_angular_diff(current_angle, die_angles[to_idx])
        rot_time = rot_diff / camera_velocity
        
        total_time += max(trans_time , rot_time)
        
        
        current_angle = die_angles[to_idx]
    
    return total_time


start_idx = 0
remaining_indices = list(range(1, len(centers)))

best_time = float('inf')
best_path = None

for perm in permutations(remaining_indices):
    path = [start_idx] + list(perm)
    time = path_total_time(path)
    if time < best_time:
        best_time = time
        best_path = [centers[i] for i in path]  

ans = {
    "TotalTime": round(best_time, 3),
    "Path": best_path
}

with open("TestCase_2_4.json", "w") as f:
    json.dump(ans, f, indent=4)

print(f"Best Total Time: {best_time:.3f} seconds")