from time import time
import math


class Reward:
    def __init__(self, verbose=False):
        self.previous_steps = None
        self.initial_time = None
        self.verbose = verbose

    @staticmethod
    def get_time(params):
        # remember: this will not return time before
        # the first step has completed so the total
        # time for lap will be lower by about 0.2s
        return params.get('timestamp', None) or time()

    def reward_function(self, params):
        if self.previous_steps is None \
                or self.previous_steps > params['steps']:
            # new lap!
            self.initial_time = self.get_time(params)
        else:
            # we're continuing a lap
            pass

        # I think actual reward function stars here
        import math

        all_wheels_on_track = params['all_wheels_on_track']
        distance_from_center = params['distance_from_center']
        track_width = params['track_width']
        is_left_of_center = params['is_left_of_center']
        is_reversed = params['is_reversed']
        speed = params['speed']
        progress = params['progress']
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        heading = params['heading']
        steering = abs(params['steering_angle']) # Only need the absolute steering angle
        heading = params['heading']*3.14/180 # TODO: Remove temp fix
        steps = params['steps']

        y = params['y']
        x = params['x']
        if abs(heading) < 45:
            is_left_of_center = y > waypoints[closest_waypoints[0]][1]
        elif abs(heading) > 135:
            is_left_of_center = y < waypoints[closest_waypoints[0]][1]
        elif heading > 0:
            is_left_of_center = x < waypoints[closest_waypoints[0]][0]
        else:
            is_left_of_center = x > waypoints[closest_waypoints[0]][0]
           
        
        if (is_reversed):
            return 0.0

        # Initialize reward with a small number but not zero
        # because zero means off-track or crashed
        reward = 1e-3

        # Reward if the agent stays inside the two borders of the track
        #if all_wheels_on_track and (0.5 * track_width - distance_from_center) >= 0.05:
        if (0.5 * track_width - distance_from_center) >= -.1:
            reward_lane = 1.0
        else:
            reward_lane = 1e-3


        # Calculate reward by putting different weights on 
        # the two aspects above
        reward = 4.0 * reward_lane

        CURVE_DISTANCE_THRESHOLD = 1
        CURVE_ANGLE_DETECTION = 10
        curve_angle, is_positive, curve_distance = get_next_curve(closest_waypoints, waypoints, CURVE_ANGLE_DETECTION)
        is_opposite_to_curve = (is_positive and not is_left_of_center) or (not is_positive and is_left_of_center)
        #print(f"angle: {angle} direction: {is_positive} distance: {distance}")
        """if distance > CURVE_DISTANCE_THRESHOLD:
            if is_opposite_to_curve:
                return reward * 1000
            else:
                return reward / 10
        else:
            return reward / 10"""

        # Penalize not going fast in a straight section
        STRAIGHT_SPEED_THRESHOLD = 2
        STRAIGHT_STEERING_THRESHOLD = 9
        if is_straight(closest_waypoints, waypoints, 10, 4) and curve_distance > CURVE_DISTANCE_THRESHOLD:
            reward *= min(STRAIGHT_SPEED_THRESHOLD, speed)
            if is_opposite_to_curve:
                reward *= 2
            """if speed < STRAIGHT_SPEED_THRESHOLD:
                reward *= 0.75"""
            if steering > STRAIGHT_STEERING_THRESHOLD:
                reward *= 0.85
        elif not is_opposite_to_curve: #and distance_from_center > track_width*0.25:
            reward *= 2 # High reward for taking a closed curve
        else:
            reward /= speed
            reward *= min(1, speed)
        
        """track_direction = get_waypoints_angle_degrees(closest_waypoints[0], closest_waypoints[1], waypoints)
        
        # Calculate the difference between the track direction and the heading direction of the car
        direction_diff = abs(track_direction - heading)
        if direction_diff > 180:
            direction_diff = 360 - direction_diff

        # Penalize the reward if the difference is too large
        DIRECTION_THRESHOLD = 40.0
        if direction_diff > DIRECTION_THRESHOLD:
            #print(direction_diff)
            reward *= 0.5"""
        
        # Steering penality threshold, change the number based on your action space setting
        ABS_STEERING_THRESHOLD = 15 

        # Penalize reward if the car is steering too much
        if steering > ABS_STEERING_THRESHOLD:
            reward *= 0.9

        """if steps > 300:
            reward *= 1 - (steps-300)/100"""

        # And I think it ends here
        self.previous_steps = params['steps']

        if self.verbose:
            print(params)

        return reward


def is_straight(closest_waypoints, waypoints, angle_threshold=10, node_count=3):
    if node_count < 3:
        return True
    waypoint_count = len(waypoints)
    first_node = closest_waypoints[0]
    second_node = closest_waypoints[1]
    last_node = (first_node + node_count) % waypoint_count
    angle_closest = get_waypoints_angle_degrees(first_node, second_node, waypoints)
    angle_last = get_waypoints_angle_degrees(first_node, last_node, waypoints)
    return abs(angle_closest - angle_last) < angle_threshold


def get_next_curve(closest_waypoints, waypoints, threshold_angle=10):
    waypoint_count = len(waypoints)
    previous_node = closest_waypoints[0]
    next_node = closest_waypoints[1]
    current_angle = get_waypoints_angle_degrees(previous_node, next_node, waypoints)
    angle_diff = 0
    while angle_diff < threshold_angle:
        next_node = (next_node + 1) % waypoint_count
        new_angle = get_waypoints_angle_degrees(previous_node, next_node, waypoints)
        angle_diff = new_angle - current_angle
        direction = angle_diff > 0
        angle_diff = abs(angle_diff)
        if angle_diff > 180:
            angle_diff = 360 - angle_diff
    x1, y1 = waypoints[previous_node]
    x2, y2 = waypoints[next_node]
    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return angle_diff, direction, distance
    

def get_waypoints_angle_degrees(first_node, second_node, waypoints):
    # Calculate the direction of the center line based on the closest waypoints
    prev_point = waypoints[first_node]
    next_point = waypoints[second_node]

    # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
    track_direction = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0]) 
    # Convert to degree
    return math.degrees(track_direction)

reward_object = Reward()


def reward_function(params):
    return reward_object.reward_function(params)
