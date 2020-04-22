class GlobalMap():

    def __init__(self):
        self.walls = []

    def __getitem__(self, key):
        if isinstance(key, (int, long)):
            return self.walls[int(key)]
        return None
