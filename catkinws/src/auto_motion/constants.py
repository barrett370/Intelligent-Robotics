
class Constants:
    def __init__(self, sim_mode):
        self.sim_mode = sim_mode # sets the vairables if in sim mode
        # self.flip = 1
        # self.turn = False

        # self.right_avg = 0
        self.RIGHT = -90
        self.LEFT = 90
        self.BACKWARDS = 180
        self.FORWARD = 0
        # self.average_count = 0
        # self.sum_readings = []
        # self.history = [self.FORWARD, self.FORWARD, self.FORWARD]
        # self.desired_bearing = 0
        self.RATE = 3  # no. calcs in average
        self.HZ = 10
        # correction = False
        # self.move_and_turn = False
        print('sim_mode: '+ str(False))
        if self.sim_mode:
            # Laser groupings
            self.LEFT_LOWER = 0
            self.LEFT_UPPER = 50
            self.CENTRE_LEFT_LOWER = 200
            self.CENTRE_LEFT_UPPER = 224
            self.CENTRE_LOWER = 224
            self.CENTRE_UPPER = 275
            self.CENTRE_RIGHT_LOWER = 300
            self.CENTRE_RIGHT_UPPER = 375
            self.RIGHT_LOWER = 450
            self.RIGHT_UPPER = 500
            self.MAX_RANGE = 3
        else:
            # Laser groupings
            self.RIGHT_LOWER = 25
            self.RIGHT_UPPER = 125
            self.CENTRE_RIGHT_LOWER = 150
            self.CENTRE_RIGHT_UPPER = 250
            self.CENTRE_LOWER = 325
            self.CENTRE_UPPER = 375
            self.CENTRE_LEFT_LOWER = 376
            self.CENTRE_LEFT_UPPER = 426
            self.LEFT_LOWER = 675
            self.LEFT_UPPER = 700

            # Optimal Ranges
            self.RIGHT_OPTIMAL = 0.5
            self.RIGHT_MAX = 1.5
            self.RIGHT_MIN = 1.0
            self.FRONT_MIN = 1.5
            self.LEFT_MIN = 1.0
            self.MAX_RANGE = 5.5
