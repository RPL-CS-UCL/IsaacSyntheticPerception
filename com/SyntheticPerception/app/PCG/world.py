import json

class Environment:
    def __init__(self):
        self.regions = []

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

class EnvParamReader:

    def __init__(self):

        self._file_path = ""
        self.environments = []

    # Finish this function
    def JSON_to_obj(self):
        with open('self._file_path', 'r') as f:
          data = json.load(f)

          environment = None
          for environment in data["Enironment"]:
             environment = Environment() 
              for region in environment["Regions"]:
                  for zone in region["Zones"]:
                      for xObject in zone["XObjects"]:

          



"""

{
  "Environment": {
    "Regions": {
      "Zones": {
        "XObject": {
          "name": "name",
          "position": "pos",
          "orientation": "ori",
          "poisson_size": "size"
        }
      }
    }
  }
}

"""
