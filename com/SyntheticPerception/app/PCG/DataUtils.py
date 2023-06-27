import json

class PCGObject:
    def __init__(self) -> None:
         
        self.prim = None
        self.prim_path = None
        self.object_scale = 1
        self.object_scale_delta = 0
        self.allow_y_rot = False
        self.unique_id = ''
        self.usd_path = ''
        self.class_name = ''
        self.poisson_size = 1
        pass

    def get_y_rot_state(self):
        if self.allow_y_rot:
            return 'Enabled'
        return 'Disabled'

    def __str__(self) -> str:
        return f'prim: {self.prim} \n prim_path: {self.prim_path}\n Object Scale: {self.object_scale}\n \
                object scale delta: {self.object_scale_delta}\n allow y rot: {self.allow_y_rot}\n usdpath: {self.usd_path}\n unique_id: {self.unique_id}'

class PCGObjectReader:
    def __init__(self) -> None:
        self.objects = []


    def read(self, path):

        with open(path, 'r+') as infile:
            try:
                data = json.load(infile)
                


                return True

            except:
                return False
    def write(self, path, object):
        data = {}
        with open(path, 'w+') as outfile:
            specific_data = {
                'object_scale':object.object_scale,
                'object_scale_delta': object.object_scale_delta,
                'poisson_size' :object.posson_size,
                'allow_y_rot':object.allow_y_rot,
                'class_name':object.class_name,
                'usd_path':object.usd_path,
            }
            # data[local_selected.unique_id]=
            json.dump(data, outfile)



