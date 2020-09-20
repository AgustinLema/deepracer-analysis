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
        
        reward = get_reward(params)

        # And I think it ends here
        self.previous_steps = params['steps']

        if self.verbose:
            print(params)

        return reward

def get_reward(params):
    all_wheels_on_track = params['all_wheels_on_track']
    distance_from_center = params['distance_from_center']
    track_width = params['track_width']
    is_left_of_center = params['is_left_of_center']
    speed = params['speed']
    progress = params['progress']
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    heading = params['heading']
    steering = abs(params['steering_angle']) # Only need the absolute steering angle
    steps = params['steps']
    is_reversed = params['is_reversed']
    is_offtrack = distance_from_center > (0.5 * track_width)#params['is_offtrack']

    
    steering = steering*3.14/180 # TODO: Remove temp fix
    heading = params['heading']*3.14/180 # TODO: Remove temp fix
    y = params['y']
    x = params['x']
    # If heading is more than 90 degrees, we are going left
    # If heading is positive we go up
    if abs(heading) < 45:
        is_left_of_center = y > waypoints[closest_waypoints[0]][1]
    elif abs(heading) > 135:
        is_left_of_center = y < waypoints[closest_waypoints[0]][1]
    elif heading > 0:
        is_left_of_center = x < waypoints[closest_waypoints[0]][0]
    else:
        is_left_of_center = x > waypoints[closest_waypoints[0]][0]


    # Actual logic starts here

    if (0.5 * track_width - distance_from_center) >= -.1 or is_reversed or is_offtrack:
        reward = 1.0
    else:
        reward = 1e-3

    MAX_SPEED = 4
    MAX_SPEED_CURVES_TURNING = 1.3
    MAX_SPEED_CURVES_TURNING_CEILING = 2.5 # Always higher than max speed curves turning

    if speed > MAX_SPEED *.66:
        speed_delta = speed - (MAX_SPEED *.66)
        speed_multiplier = 1.66 + (speed_delta/MAX_SPEED)  # Range 1.66 to 2
    elif speed >= MAX_SPEED *.33:
        speed_delta = speed - (MAX_SPEED *.33)
        speed_multiplier = 1 + (speed_delta/MAX_SPEED) * 2  # Range 1 to 1.66
    else:
        speed_multiplier = (speed/MAX_SPEED) * 3 # Range 0 to 1
    
    reward *= speed_multiplier

    steering_multipliers = [0.93, 0.95, 0.97, 1]

    STEERING_STEPS = 10 * 0.90 # Small correction since it is not exact
    if steering >= STEERING_STEPS * 3:
        steering_action_idx = 0
    elif steering >= STEERING_STEPS * 2:
        steering_action_idx = 1
    elif steering >= STEERING_STEPS:
        steering_action_idx = 2
    else:
        steering_action_idx = 3

    steering_reward = steering_multipliers[steering_action_idx]
    
    # The faster it goes, higher the risk for turning
    if speed > MAX_SPEED_CURVES_TURNING:
        steering_overspeed = speed - MAX_SPEED_CURVES_TURNING
        max_steering_overspeed = MAX_SPEED - MAX_SPEED_CURVES_TURNING
        steering_overspeed_factor = 1 + ((steering_overspeed / max_steering_overspeed) * 9) # 1 to 10
        steering_punishment = (1 - steering_reward)
        steering_reward = 1 - (steering_punishment * steering_overspeed_factor) # Max speed = ten times punishment while steering
        
    
    reward *= steering_reward
    
    CURVE_DISTANCE_THRESHOLD = 1
    CURVE_ANGLE_DETECTION = 10
    curve_angle, is_positive, curve_distance = get_next_curve(closest_waypoints, waypoints, CURVE_ANGLE_DETECTION)
    is_opposite_to_curve = (is_positive and not is_left_of_center) or (not is_positive and is_left_of_center)
    
    if curve_distance < CURVE_DISTANCE_THRESHOLD:
        if not is_opposite_to_curve:
            reward = reward * 1.2
        elif speed > MAX_SPEED_CURVES_TURNING:
            overspeed = speed - MAX_SPEED_CURVES_TURNING  # How much faster than limit
            max_overspeed = MAX_SPEED_CURVES_TURNING_CEILING - MAX_SPEED_CURVES_TURNING  # What is the max speed difference to reach full punishment
            overspeed_percentage = min(1, overspeed / max_overspeed)
            reward *= (1 - overspeed_percentage) # If going faster than turn speed, punish. if max_turning_ceiling=2.5 and max_turning=1.3. Going 1.5 would mean (0.2/1.2 punishment = 0.16%)
            
    
    MAX_POSSIBLE_REWARD = 2.4
    
    normalized_reward = (reward / MAX_POSSIBLE_REWARD) * 20

    return float(normalized_reward)


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
    prev_point = waypoints[first_node]
    next_point = waypoints[second_node]
    track_direction = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0]) 
    return math.degrees(track_direction)