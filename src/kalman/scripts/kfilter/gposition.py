class GlobalPosition():

    def __init__(self, x_0=0, y_0=0, theta_0=0):
        self.x = x_0
        self.y = y_0
        self.theta = theta_0

    def set_position(self, x=None, y=None, theta=None):
        self.x = x if x is not None else self.x
        self.y = y if y is not None else self.y
        self.theta = theta if theta is not None else self.theta