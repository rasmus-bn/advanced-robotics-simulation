from shapely import Point, LineString
from shapely.ops import nearest_points
import math
import sys

class LandmarkHandler:
    def __init__(self) -> None:
        self.landmarks = {}
        self.landmark_id = 0

   

    def find_landmarks(self, intersect_points, walls, current_pose, threshold=5):
        landmark_sightings = []
        #insert code here
        return landmark_sightings
    
    
    #creates and stores a new landmark if it has not already been spottet
    #in real life, this would also be where you would check if any line segments belonged to the same landmark/wall.
    def create_landmark(self, wall, threshold=5):
        for existing_landmark_key in self.landmarks:
            existing_landmark = self.landmarks[existing_landmark_key]
            if existing_landmark['wall_segment'].distance(wall) < threshold:
                return existing_landmark['id']
        #if it doesnt already exist, we create it
        self.landmark_id += 1
        new_landmark = {
                'wall_segment':wall,
                'id':self.landmark_id
        }
        self.landmarks[self.landmark_id] = new_landmark
        return new_landmark['id']
    

    #creates a landmark sighting
    def create_landmark_sighting(self, landmark_id, current_pose):
        wall = self.landmarks[landmark_id]['wall_segment']
        semi_orthogonal_point, bearing = self.orthogonal_projection(current_pose, wall)
        current_pose_as_point = Point(current_pose.x, current_pose.y)
        distance = current_pose_as_point.distance(semi_orthogonal_point)
        landmark_sighting = {
                "landmark_id": landmark_id,
                "point_of_intercept": (semi_orthogonal_point.x, semi_orthogonal_point.y),
                "orthogonal_distance": distance,
                "bearing_angle": bearing
            }
        return landmark_sighting
        
    def points_belong_to_straight_line(self, p1, p2, p3):
        # Extract coordinates
        x1, y1 = p1.x, p1.y
        x2, y2 = p2.x, p2.y
        x3, y3 = p3.x, p3.y
        
        # Calculate the cross product of vectors (p1p2) and (p1p3)
        cross_product = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1)
        
        # If cross product is zero, the points are collinear
        return cross_product == 0
    

    #NOTE: this doesn't really create the orthogonal projection if the point lies outside of the line segment
    #but it is easy to understand and will be identical for landmarks found with other lidar rays that intersect with
    #the same wall.
    def orthogonal_projection(self, current_pose, linestring):
        point = Point(current_pose.x, current_pose.y)
        theta =  current_pose.theta

        # Find the nearest point on the LineString to the given point
        nearest_point_on_line = nearest_points(point, linestring)[1]

        # Calculate the bearing angle between the robot's current position and the orthogonal intercept point
        bearing = math.atan2(nearest_point_on_line.y - point.y, nearest_point_on_line.x - point.x) - theta

        return nearest_point_on_line, bearing
    
  