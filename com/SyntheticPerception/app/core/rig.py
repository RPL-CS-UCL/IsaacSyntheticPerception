from objects import Object


class Rig(Object):
    def __init__(self, position, rotation, scale, usd_path, prim_name, parent_path, stage, semantic_class="None") -> None:
        super().__init__(position, rotation, scale, usd_path, prim_name, parent_path, stage, semantic_class)

        # init rig related stuff
        # add rigid body with certain collider
