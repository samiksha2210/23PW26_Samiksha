from itertools import permutations
import json
import math
with open ("Input_Milestone2_Testcase3.json","r") as f:
    data=json.load(f)


velocity=data['StageVelocity']
initial_pos=data['InitialPosition']
Dies=data['Dies']


centers=[initial_pos]
for i in range(len(Dies)):
    ind=0
    x=(Dies[i]['Corners'][ind][0]+Dies[i]['Corners'][ind+1][0]+Dies[i]['Corners'][ind+2][0]+Dies[i]['Corners'][ind+3][0])/4
    y=(Dies[i]['Corners'][ind][1]+Dies[i]['Corners'][ind+1][1]+Dies[i]['Corners'][ind+2][1]+Dies[i]['Corners'][ind+3][1])/4
    centers.append([x,y])

def dist(p1,p2):
    x1,y1=p1[0],p1[1]
    x2,y2=p2[0],p2[1]

    return math.sqrt((x1-x2)**2+(y1-y2)**2)
total_dist=0



start_point = centers[0]
remaining_points = centers[1:]

shortest_distance = float('inf')
best_path = []
num_points=len(centers)
for perm in permutations(remaining_points):
    current_path = [start_point] + list(perm)
    current_distance = 0
    
    for i in range(num_points - 1):
        current_distance += dist(current_path[i], current_path[i+1])
        
    if current_distance < shortest_distance:
        shortest_distance = current_distance
        best_path = current_path




def calculate_consecutive_distances(points):
   
    distances = []
    for i in range(len(points) - 1):
        p1 = points[i]
        p2 = points[i+1]
        distance = math.dist(p1, p2)
        distances.append(distance)
    return distances

consecutive_dist=calculate_consecutive_distances(best_path)

time=0
for i in range(len(consecutive_dist)):
    time+=consecutive_dist[i]/velocity



ans={"TotalTime":round(time,3),"Path":best_path}


with open("TestCase_2_3.json","w") as f:
    json.dump(ans,f,indent=4)
