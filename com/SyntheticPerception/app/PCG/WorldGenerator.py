import asyncio
import random

import omni
import numpy as np
from .AreaMaskGenerator import generate_world_from_file

from omni.isaac.core.utils.semantics import get_semantics
from pxr import (
    UsdGeom,
    Gf,
    UsdPhysics,
    Semantics,
)  # pxr usd imports used to create cube

from pxr import Usd, Gf
from omni.isaac.core.utils.stage import (
    add_reference_to_stage,
    is_stage_loading,
    update_stage_async,
    update_stage,
)

from pxr import UsdShade, Sdf
import math



class WorldManager:
    def __init__(self) -> None:
        self.__undefined_class_string = "NAN"
        self.occupancy = []
        self._o = "[World generator] "

    def add_semantic(self, p, prim_class):
        """Adds semantic to prim"""
        sem_dict = get_semantics(p)
        collisionAPI = UsdPhysics.CollisionAPI.Apply(p)
        if 'Semantics' not in sem_dict:
            # print(
            #     'adding semantics and collider to ',
            #     p.GetPrimPath(),
            #     ' of class ',
            #     prim_class,
            # )
            sem = Semantics.SemanticsAPI.Apply(p, 'Semantics')
            sem.CreateSemanticTypeAttr()
            sem.CreateSemanticDataAttr()
            sem.GetSemanticTypeAttr().Set('class')
            sem.GetSemanticDataAttr().Set(prim_class)

    def __add_semantics_to_all2(self, stage):
        """Add semantic information to all prims on stage based on parent xform"""
        prim_class = self.__undefined_class_string
        completed_classes = []
        for prim_ref in stage.Traverse():
            prim_ref_name = str(prim_ref.GetPrimPath())
            len_of_prim = len(prim_ref_name.split('/'))
            for word in prim_ref_name.split('/'):

                if 'class' in word and word not in completed_classes:

                    prim_class = word

                    # self.add_semantic(prim_ref, prim_class)
                    for i in range(len(prim_ref.GetChildren())):
                        prim_child = prim_ref.GetChildren()[i]
                        len_of_child = len(
                            str(prim_child.GetPrimPath()).split('/')
                        )
                        # print(len_of_prim, ' : ', len_of_child)
                        if abs(len_of_prim - len_of_child) == 1:
                            # print(prim_child)
                            self.add_semantic(prim_child, prim_class)

                    completed_classes.append(prim_class)
    def spawn_asset(
        self,
        asset_path,
        class_name,
        prim_name,
        x,
        y,
        z,
        scale,
        object_scale_delta,
        allow_rot,
        orign_p_name = "",
        override=False,
        rot = (0,0,0),
    ):

        prim_path = '/World/' + 'class_' + class_name + '/' + prim_name

        # if not override:
        add_reference_to_stage(usd_path=asset_path, prim_path=prim_path)
        
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        # prim.GetReferences().AddReference(assetPath=asset_path, primPath=prim_path)
        prim.SetInstanceable(True)

        collisionAPI = UsdPhysics.CollisionAPI.Apply(prim)
        sem = Semantics.SemanticsAPI.Apply(prim, 'Semantics')
        sem.CreateSemanticTypeAttr()
        sem.CreateSemanticDataAttr()
        sem.GetSemanticTypeAttr().Set('class')
        sem.GetSemanticDataAttr().Set(class_name)


        # omni.kit.commands.execute('CopyPrim',
        #     path_from=orign_p_name,
        #     path_to=prim_path,
        #     duplicate_layers=False,
        #     combine_layers=False,
        #     exclusive_select=False,
        #     flatten_references=False,
        #     copy_to_introducing_layer=False)
        # here we want to modify the scale
        low_lim = scale - object_scale_delta
        high_lim = scale + object_scale_delta
        scale = random.uniform(low_lim, high_lim) #/ 100

        random_rotation = 0
        if allow_rot:
            random_rotation = random.uniform(0, 360)


        # omni.kit.commands.execute('CreatePayloadCommand',
        #     usd_context=omni.usd.get_context(),
        #     path_to=Sdf.Path(prim_path),
        #     asset_path=asset_path,
        #     instanceable=True)
        omni.kit.commands.execute(
            'TransformPrimSRTCommand',
            path=prim_path,  # f"/World/{p_name}",
            old_scale=Gf.Vec3f(1.0, 1.0, 1.0),
            new_scale=Gf.Vec3f(scale, scale, scale),
            old_translation=Gf.Vec3f(x, y, z),
            new_translation=Gf.Vec3f(x, y, z),
            old_rotation_euler=Gf.Vec3f(0, 0, 0),
            old_rotation_order=Gf.Vec3i(0, 1, 2),
            # new_rotation_euler=Gf.Vec3f(0, 0, random_rotation),

            new_rotation_euler=Gf.Vec3f(rot[0], rot[1], random_rotation),
            new_rotation_order=Gf.Vec3i(0, 1, 2),
            time_code=Usd.TimeCode(),
            had_transform_at_key=False,
        )
        omni.kit.commands.execute(
            'TransformPrimSRTCommand',
            path=prim_path,  # f"/World/{p_name}",
            old_scale=Gf.Vec3f(1.0, 1.0, 1.0),
            new_scale=Gf.Vec3f(scale, scale, scale),
            old_translation=Gf.Vec3f(x, y, z),
            new_translation=Gf.Vec3f(x, y, z),
            old_rotation_euler=Gf.Vec3f(0, 0, 0),
            old_rotation_order=Gf.Vec3i(0, 1, 2),
            new_rotation_euler=Gf.Vec3f(rot[0], rot[1], random_rotation),
            new_rotation_order=Gf.Vec3i(0, 1, 2),
            time_code=Usd.TimeCode(),
            had_transform_at_key=False,
        )
    def point_in_triangle(self,pt, v1, v2, v3):
        def sign(p1, p2, p3):
            return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1])

        d1 = sign(pt, v1, v2)
        d2 = sign(pt, v2, v3)
        d3 = sign(pt, v3, v1)

        has_neg = (d1 < 0) or (d2 < 0) or (d3 < 0)
        has_pos = (d1 > 0) or (d2 > 0) or (d3 > 0)

        return not (has_neg and has_pos)
    def point_in_triangle3(self, pt, A, B, C):
        xp = pt[0]
        yp = pt[1]

        x1,y1,z1, =A[0],A[1], A[2]
        x2,y2,z2, =B[0],B[1],B[2]
        x3,y3,z3, =C[0],C[1],C[2]
        c1 = (x2-x1)*(yp-y1)-(y2-y1)*(xp-x1)
        c2 = (x3-x2)*(yp-y2)-(y3-y2)*(xp-x2)
        c3 = (x1-x3)*(yp-y3)-(y1-y3)*(xp-x3)
        if (c1<0 and c2<0 and c3<0) or (c1>0 and c2>0 and c3>0):
            return True
            print("The point is in the triangle.")
        else:
            return False
            print("The point is outside the triangle.")
    def point_in_triangle2(self,pt, A, B, C):
        """
        Check if point pt lies inside the triangle defined by vertices A, B, C
        Each vertex is a tuple (x, y, z)
        """
        # Convert vertices and point to numpy arrays
        pt = np.array([pt[0],pt[1], 0])
        A = np.array([A[0], A[1], 0])
        B = np.array([B[0], B[1], 0])
        C = np.array([C[0], C[1], 0])

        # Vectors
        v0 = C - A
        v1 = B - A
        v2 = pt - A

        # Compute dot products
        dot00 = np.dot(v0, v0)
        dot01 = np.dot(v0, v1)
        dot02 = np.dot(v0, v2)
        dot11 = np.dot(v1, v1)
        dot12 = np.dot(v1, v2)

        # Compute barycentric coordinates
        invDenom = 1 / (dot00 * dot11 - dot01 * dot01)
        u = (dot11 * dot02 - dot01 * dot12) * invDenom
        v = (dot00 * dot12 - dot01 * dot02) * invDenom

        # Check if point is in triangle
        return (u >= 0) and (v >= 0) and (u + v < 1)
    def calculate_height(self,x, y, A, B, C):
        """
        Calculate the height at point (x, y) within a triangle with vertices A, B, C
        Each vertex is a tuple (x, y, z)
        """

        # Convert vertices to numpy arrays for vector operations
        A = np.array(A)
        B = np.array(B)
        C = np.array(C)
        P = np.array([x, y, 0])  # We don't know the z-coordinate yet

        # Vectors for the edges of the triangle
        v0 = B - A
        v1 = C - A
        v2 = P - A

        # Compute dot products
        dot00 = np.dot(v0, v0)
        dot01 = np.dot(v0, v1)
        dot02 = np.dot(v0, v2)
        dot11 = np.dot(v1, v1)
        dot12 = np.dot(v1, v2)

        # Compute barycentric coordinates
        invDenom = 1 / (dot00 * dot11 - dot01 * dot01)
        alpha = (dot11 * dot02 - dot01 * dot12) * invDenom
        beta = (dot00 * dot12 - dot01 * dot02) * invDenom
        gamma = 1 - alpha - beta

        # Interpolate the height using the barycentric coordinates
        z = alpha * A[2] + beta * B[2] + gamma * C[2]

        return z
    def barycentric_weights(self,x, y, A, B, C):
        """
        Calculate the barycentric weights for a point (x, y) within a triangle defined by vertices A, B, and C.
        Each vertex is a tuple (x, y, z).
        """
        def tri_area(P1, P2, P3):
            return 0.5 * np.linalg.det([[P1[0], P1[1], 1],
                                        [P2[0], P2[1], 1],
                                        [P3[0], P3[1], 1]])

        area_ABC = tri_area(A, B, C)
        weight_A = tri_area((x, y), B, C) / area_ABC
        weight_B = tri_area(A, (x, y), C) / area_ABC
        weight_C = tri_area(A, B, (x, y)) / area_ABC

        return weight_A, weight_B, weight_C

    def interpolate_z(self,x, y, A, B, C):
        """
        Interpolate the z-value at a point (x, y) inside a triangle defined by vertices A, B, and C.
        """
        weight_A, weight_B, weight_C = self.barycentric_weights(x, y, A, B, C)
        z = weight_A * A[2] + weight_B * B[2] + weight_C * C[2]
        return z
    def calculate_normal(self,A, B, C):
        """
        Calculate the normal vector of a triangle defined by vertices A, B, and C.
        Each vertex is a tuple (x, y, z).
        """
        # Convert vertices to numpy arrays
        A = np.array(A)
        B = np.array(B)
        C = np.array(C)

        # Calculate two vectors in the plane of the triangle
        vector1 = B - A
        vector2 = C - A

        # Calculate the cross product, which is perpendicular to the plane of the triangle
        normal = np.cross(vector1, vector2)

        # Normalize the vector to have a length of 1
        normal_unit = normal / np.linalg.norm(normal)

        return normal_unit


    def axis_angle_to_euler(self,vertical_normal, target_normal):
        """
        Calculate the Euler angles to rotate 'vertical_normal' to 'target_normal'.
        """
        # Ensure normals are unit vectors
        vertical_normal = vertical_normal / np.linalg.norm(vertical_normal)
        target_normal = target_normal / np.linalg.norm(target_normal)

        # Axis of rotation (cross product)
        axis = np.cross(vertical_normal, target_normal)

        # Angle of rotation (dot product and arccos)
        angle = np.arccos(np.dot(vertical_normal, target_normal))
        print(axis,angle)
# Convert to quaternion
        quaternion = self.axis_angle_to_quaternion(axis, angle)

# Convert to Euler angles
        euler_angles = self.quaternion_to_euler(quaternion)
        euler_deg = []
        for rad in euler_angles:
            euler_deg.append(math.degrees(rad))
        print("Euler Angles:", euler_deg)
        return euler_deg

    def axis_angle_to_quaternion(self,axis, angle):
        """
        Convert an axis-angle rotation to a quaternion.
        """
        axis = axis / np.linalg.norm(axis)  # Normalize the axis
        qx = axis[0] * np.sin(angle / 2)
        qy = axis[1] * np.sin(angle / 2)
        qz = axis[2] * np.sin(angle / 2)
        qw = np.cos(angle / 2)
        return np.array([qw, qx, qy, qz])

    def quaternion_to_euler(self,quaternion):
        """
        Convert a quaternion to Euler angles.
        """
        qw, qx, qy, qz = quaternion
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (qw * qy - qz * qx)
        if np.abs(sinp) >= 1:  # Use 90 degrees if out of range
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return np.array([roll, pitch, yaw])
    def spawn_loop(
        self,
        path,
        class_name,
        p_name,
        coll,
        height_map,
        scale=1,
        object_scale_delta=0,
        allow_rot=True,
    ):

        for i, n in enumerate(coll):
            override=False
            # if i == 1:
            #
            #     prim_path = '/World/' + 'class_' + class_name + '/' + p_name 
            #
            #     add_reference_to_stage(usd_path=path, prim_path=prim_path)
            #
            #     override=True
        
            x, y = n
            x = float(x)
            y = float(y)
            mesh_scale = 10
            x_ind = x * mesh_scale 
            y_ind = y * mesh_scale
            # mesh_height_modifier = 10
            # if x_ind >= 2560:
            #     print('x, overfilled', x_ind)
            #     x_ind = 2559
            # if y_ind >= 2560:
            #
            #     print('y, overfilled', y_ind)
            #     y_ind = 2559

            # get the closest vertex (x,y)
            closest_x = int(round(x_ind/10))
            closest_y = int(round(y_ind/10))
            closest = (closest_y, closest_x)

            closest_ind =np.ravel_multi_index(closest, (len(height_map), len(height_map))) 

            right = (closest_y +1, closest_x)
            right_ind =np.ravel_multi_index(right, (len(height_map), len(height_map))) 
            right = (closest_y +1, closest_x,0)

            left = (closest_y -1, closest_x) 
            left_ind =np.ravel_multi_index(left, (len(height_map), len(height_map))) 

            left = (closest_y -1, closest_x,0) 
            up= (closest_y, closest_x+1) 
            up_ind =np.ravel_multi_index(up, (len(height_map), len(height_map))) 

            up= (closest_y, closest_x+1,0) 
            down= (closest_y, closest_x-1) 
            down_ind =np.ravel_multi_index(down, (len(height_map), len(height_map))) 

            down= (closest_y, closest_x-1,0) 
            possible_inds = [[up, right], [right, down], [down,left],[left,up]]

            z = float(height_map[int(round(y_ind/10))][int(round(x_ind/10))])# / mesh_height_modifier   # was abs
            for inds in possible_inds:
                vert1 = inds[0]
                vert1= (vert1[1],vert1[0], height_map[vert1[0]][vert1[1]])

                vert2 = inds[1]
                vert2= (vert2[1],vert2[0],height_map[vert2[0]][vert2[1]])

                c = (closest[1],closest[0],height_map[closest[0]][closest[1]])
                if self.point_in_triangle3((x,y),c,vert1,vert2):
                    print("THIS POINT IS IN THIS TRIANGLE HERE")
                    print(f" x, {x}, y{y}, v1, {vert1}, v2 {vert2}, c {c}, zbefore, {z}")
                    z = self.calculate_height(x,y,c,vert1,vert2 ) 

                    print(f" now its {z}")
                    x1,y1,z1, = vert1[1], vert1[0], vert1[2]
                    x2,y2,z2, = vert2[1], vert2[0], vert2[2]
                    x3,y3,z3, =closest[1],closest[0],c[2]
                    z = (z3*(x-x1)*(y-y2) + z1*(x-x2)*(y-y3) + z2*(x-x3)*(y-y1) - z2*(x-x1)*(y-y3) - z3*(x-x2)*(y-y1) - z1*(x-x3)*(y-y2))/(  (x-x1)*(y-y2) +   (x-x2)*(y-y3) +   (x-x3)*(y-y1) -   (x-x1)*(y-y3) -   (x-x2)*(y-y1) -   (x-x3)*(y-y2))
                    print(f" now its {z}")

                    z = self.interpolate_z(x,y,c,vert1,vert2 ) -0.1
                    normal = self.calculate_normal(vert1,vert2,c)
                    euler_deg = self.axis_angle_to_euler([0,0,1],normal)
                    euler_deg[0] = max(euler_deg[0], 5)
                    euler_deg[1] = max(euler_deg[1], 5)
                    break
                else:
                    print(" =============== this triangle was not detected ")
                    z = 150

                    print(f" now its {z}")

            print("test out ",self.interpolate_z(2077.2607421875, 261.5051574707031,  (2077, 261, 0.0030516041970591323),  (2078, 262, 0.12388602059223004),  (2077, 262, 0.12202789148699979)))



            cc =(int(round(y_ind/10)),int(round(x_ind/10)) )
            ind = np.ravel_multi_index(cc, (len(height_map), len(height_map)))
            # print(np.asarray(self.t_normals))
            poss_rot = np.asarray(self.t_normals)[ind]
            # print("triangle normals")
            # print(poss_rot)
            # second one is iterated fasted
            if self.occupancy[int(y_ind/10)][int(x_ind/10)] != 0:
                # print("skipping oj spawn")
                continue

            self.occupancy[int(y_ind/10)][int(x_ind/10)]= 1 
            _p_name = f'{p_name}_{i}'
            self.spawn_asset(
                path,
                class_name,
                _p_name,
                x,
                y,
                z,
                scale,
                object_scale_delta,
                allow_rot,
                override = override,
                orign_p_name = p_name,
                rot = euler_deg#poss_rot
            )

    def create_terrains(self, terrain_info):
        print("trying to create terrains")

        # create the parent

        omni.kit.commands.execute(
            'CreatePrimWithDefaultXform',
            prim_type='Xform',
            prim_path='/World/t',
            attributes={},
            select_new_prim=True,
        )

        for key in terrain_info:
            mesh_path = terrain_info[key].mesh_path
            scale = terrain_info[key].scale
            mat_path = terrain_info[key].material_path
            mat_name = mat_path.split('/')[-1]
            mat_name = mat_name.replace('.mdl', '')
            mesh_path = mesh_path.replace('.obj', '.usd')
            # spawn prim

            prim_p = f'/World/t/class_{mat_name}'
            # prim_p = f'/World/t/terrain{key}'

            stage = omni.usd.get_context().get_stage()
            scale = 1#0.01
            # X SCALE SHOULD BE NEGATIVE TO FLIP IT CORRECTLY
            random_rotation = 0.0
            x, y, z = 0, 0, 0
            add_reference_to_stage(usd_path=mesh_path, prim_path=prim_p)
            self.create_material_and_bind(
                mat_name, mat_path, prim_p, scale, stage
            )
            prim=stage.GetPrimAtPath(prim_p)
            collisionAPI = UsdPhysics.CollisionAPI.Apply(prim)
            sem = Semantics.SemanticsAPI.Apply(prim, 'Semantics')
            sem.CreateSemanticTypeAttr()
            sem.CreateSemanticDataAttr()
            sem.GetSemanticTypeAttr().Set('class')
            sem.GetSemanticDataAttr().Set(mat_name)

        scale = 1#0.1
        random_rotation = 0.0
        x, y, z = 0, 0, 0
        # stage = self.usd_context.get_stage()

        omni.kit.commands.execute(
            'TransformPrimSRTCommand',
            path=f'/World/t',
            old_scale=Gf.Vec3f(1.0, 1.0, 1.0),
            new_scale=Gf.Vec3f(scale, scale, scale),
            old_translation=Gf.Vec3f(x, y, z),
            new_translation=Gf.Vec3f(x, y, z),
            # old_rotation_euler=Gf.Vec3f(-90, 0, 0),
            # old_rotation_order=Gf.Vec3i(0, 1, 2),
            # new_rotation_euler=Gf.Vec3f(-90, 0, -180),
            # new_rotation_order=Gf.Vec3i(0, 1, 2),
            time_code=Usd.TimeCode(),
            had_transform_at_key=False,
        )
        omni.kit.commands.execute(
            'TransformPrimSRTCommand',
            path=f'/World/t',
            old_scale=Gf.Vec3f(1.0, 1.0, 1.0),
            new_scale=Gf.Vec3f(scale, scale, scale),
            old_translation=Gf.Vec3f(x, y, z),
            new_translation=Gf.Vec3f(x, y, z),
            # old_rotation_euler=Gf.Vec3f(-90, 0, 0),
            # old_rotation_order=Gf.Vec3i(0, 1, 2),
            # new_rotation_euler=Gf.Vec3f(-90, 0, -180),
            # new_rotation_order=Gf.Vec3i(0, 1, 2),
            time_code=Usd.TimeCode(),
            had_transform_at_key=False,

        )
        print("terrains finished")

    def spawn_all(self, obs_to_spawn, object_dict, height_map, normals):
        self.t_normals = normals
        length = len(obs_to_spawn)
        print("trying to spawn ", length)
        counter = 1
        for key in obs_to_spawn:

            obj = object_dict[key]
            path = object_dict[key].usd_path
            print(f"{self._o} Spawning {len(obs_to_spawn[key])} of {path}. {counter} / {length}")
            class_name = obj.class_name
            if class_name == '':
                class_name = obj.unique_id
            self.spawn_loop(
                path,
                class_name,
                f'{obj.unique_id}_',
                obs_to_spawn[key],
                height_map,
                scale=obj.object_scale,
                object_scale_delta=obj.object_scale_delta,
                allow_rot=obj.allow_y_rot,
            )
            print("spawned, now we wait till stage loads")
            # update_stage()
            # print("some time should have passed")
            # return
            counter += 1

    def generate_world_generator(self, obj_path, world_path):


        print("Tring to generator worldf rom file")
        (
            obs_to_spawn,
            object_dict,
            terrain_info,
            meshGen,
        ) =generate_world_from_file(obj_path, world_path)
        height_map = meshGen._points2
        self.occupancy = np.zeros((len(height_map),len(height_map)))
        self.create_terrains(terrain_info)
        meshGen.clean_up_files()
        print("Files all cleaned up, now ready to move to next step")

        return obs_to_spawn, object_dict, height_map, meshGen.normals


    def create_material_and_bind(
        self, mat_name, mat_path, prim_path, scale, stage
    ):

        obj_prim = stage.GetPrimAtPath(prim_path)
        mtl_created_list = []

        omni.kit.commands.execute(
            'CreateAndBindMdlMaterialFromLibrary',
            mdl_name=mat_path,
            mtl_name=mat_name,
            mtl_created_list=mtl_created_list,
        )
        mtl_prim = stage.GetPrimAtPath(mtl_created_list[0])
        omni.usd.create_material_input(
            mtl_prim,
            'project_uvw',
            True,
            Sdf.ValueTypeNames.Bool,
        )

        omni.usd.create_material_input(
            mtl_prim,
            'texture_scale',
            Gf.Vec2f(scale, scale),
            Sdf.ValueTypeNames.Float2,
        )
        cube_mat_shade = UsdShade.Material(mtl_prim)

        UsdShade.MaterialBindingAPI(obj_prim).Bind(
            cube_mat_shade, UsdShade.Tokens.strongerThanDescendants
        )

    def create_world(self,  world_path, obj_path):

        (
            obs_to_spawn,
            object_dict,
            height_map,
            normals
        ) =self.generate_world_generator(
            world_path, obj_path
        )
        print("now going to spawn all objects, skipping for now")
        self.spawn_all(obs_to_spawn, object_dict, height_map, normals)
        # update_stage()
        print("stage updated")
        stage = omni.usd.get_context().get_stage()

        self.__add_semantics_to_all2(stage)
        print("semantics added")
