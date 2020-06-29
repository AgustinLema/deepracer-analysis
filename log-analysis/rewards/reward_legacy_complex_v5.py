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

    if (is_reversed or is_offtrack):
            return 0.0

    if (0.5 * track_width - distance_from_center) >= -.1 or is_reversed or is_offtrack:
        reward_lane = 1.0
    else:
        reward_lane = 1e-3

    MAX_SPEED = 3
    MAX_SPEED_CURVES = 2.2
    if speed > MAX_SPEED *.66:
        speed_multiplier = 1 + (speed/MAX_SPEED)/2
    elif speed >= MAX_SPEED *.33:
        speed_multiplier = 1
    else:
        speed_multiplier = (speed/MAX_SPEED)*3
    
    reward = 4.0 * reward_lane

    CURVE_DISTANCE_THRESHOLD = 1
    CURVE_ANGLE_DETECTION = 10
    curve_angle, is_positive, curve_distance = get_next_curve(closest_waypoints, waypoints, CURVE_ANGLE_DETECTION)
    is_opposite_to_curve = (is_positive and not is_left_of_center) or (not is_positive and is_left_of_center)
    
    steering_multipliers_curves = [0.7, 0.8, 0.9, 1]
    steering_multipliers_straight = [0.3, 0.5, 0.8, 1]

    STEERING_STEPS = 10 * 0.90 # Small correction since it is not exact
    if steering >= STEERING_STEPS * 3:
        steering_action_idx = 0
    elif steering >= STEERING_STEPS * 2:
        steering_action_idx = 1
    elif steering >= STEERING_STEPS:
        steering_action_idx = 2
    else:
        steering_action_idx = 3

    #if is_straight(closest_waypoints, waypoints, 5, 2):
    curve_state = get_curve_state(closest_waypoints, waypoints, angle_threshold=2, node_count=3)
    if curve_state == "STRAIGHT":
        if curve_distance > CURVE_DISTANCE_THRESHOLD:
            reward *= speed_multiplier
            if is_opposite_to_curve:
                reward *= 1.3# TODo: Decrease once reward for speed is given 1.2
            reward *= steering_multipliers_straight[steering_action_idx]
        else:
            reward *= steering_multipliers_curves[steering_action_idx]
            reward *= 1.3 # TODO: Remove, it is to compensate that no reward is given at this point
    else:
        if speed > MAX_SPEED_CURVES:
            reward /= 1.3
            reward *= steering_multipliers_curves[steering_action_idx]**2 # High speed = more punishment turning
        else:
            reward *= steering_multipliers_curves[steering_action_idx]
        if not is_opposite_to_curve and curve_state == 'ENTERING':
            if distance_from_center > track_width*0.25:
                reward *= 1.5
            else:
                reward *= 1.2
        elif is_opposite_to_curve and curve_state == 'EXITING':
            reward *= 1.3
    
    return float(reward)

def angle_diff(angle1, angle2):
    diff = abs(angle1 - angle2)
    return diff if diff < 180 else 360 - diff

def get_curve_state(closest_waypoints, waypoints, straight_angle_threshold=10, angle_threshold=10, node_count=2):
    if node_count < 2:
        raise Exception
    waypoint_count = len(waypoints)
    previous_node = closest_waypoints[0] - 1 if closest_waypoints[0] != 0 else waypoint_count - 1
    from_node = closest_waypoints[0]
    to_node = closest_waypoints[1]
    following_node = (closest_waypoints[1] + 1) % waypoint_count
    last_node = (to_node + node_count) % waypoint_count
    angle_before = get_waypoints_angle_degrees(previous_node, from_node, waypoints)
    angle_current = get_waypoints_angle_degrees(from_node, to_node, waypoints)
    angle_following = get_waypoints_angle_degrees(to_node, following_node, waypoints)
    #angle_following2 = get_waypoints_angle_degrees(following_node, following_node2, waypoints)
    angle_last = get_waypoints_angle_degrees(following_node, last_node, waypoints)
    #print(f"Current: {angle_current}, Following: {angle_following}, Last: {angle_last}")
    if angle_diff(angle_before, angle_following) < straight_angle_threshold:
        return "STRAIGHT"
    else:
        if angle_diff(angle_current, angle_last) < 3:
            return "EXITING"
        else:
            return "ENTERING"


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

reward_object = Reward()


def reward_function(params):
    return reward_object.reward_function(params)
