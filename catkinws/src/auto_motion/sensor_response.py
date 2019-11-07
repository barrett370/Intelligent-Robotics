class Response:
    def __init__(self, centre_avg, centre_left_avg, centre_right_avg, left_avg, right_avg):
        self.left_avg = left_avg
        self.centre_left_avg = centre_left_avg
        self.centre_avg = centre_avg
        self.centre_right_avg = centre_right_avg
        self.right_avg = right_avg

    def __str__(self):
        return "left_avg: {},centre_left_avg: {},centre_avg: {},centre_right_avg: {},right_avg: {}".format(
            self.left_avg, self.centre_left_avg, self.centre_avg, self.centre_right_avg, self.right_avg)
