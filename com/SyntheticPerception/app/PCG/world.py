class Region:
    def __init__(self):

        self._zones = []

class Zone:
    def __init__(self):
        self.zone_map = [[]]
        self.XObjects = []

class XObject:

    def __init__(self):

        self.prim_path = ""
        self.position = 0
        self.orientation = 0
        self.poisson_size = 0
