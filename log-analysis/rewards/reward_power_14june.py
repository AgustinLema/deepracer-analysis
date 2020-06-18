from time import time


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
        progress = params['progress']
        steps = params['steps']

        # Make it so that each step reduces possible score by half.
        # It is always better to have higher scores sooner (by progressing faster)
        steps += 1 # Prevent smaller numbers for steps smaller than 1
        steps_multiplier = 2**-(steps/4)

        # Also add 1 to prevent using a score of 0 at the beginning
        progress += 1

        # Expected score is to increase 100 in less than 400 steps.
        # Since step is decreasing value in half, we want to make progress result in higher rewards
        # the further you are in the track. So we duplicate the score every 1/4 of progress advance
        progress_score = 2 ** (progress)

        reward = progress_score * steps_multiplier

        # Reward must be between -1e5 and 1e5. We reduce it to prevent errors.
        reward = reward / 1E4
        reward = max(reward, 0.1)

        # And I think it ends here
        self.previous_steps = params['steps']

        if self.verbose:
            print(params)

        return reward


reward_object = Reward()


def reward_function(params):
    return reward_object.reward_function(params)