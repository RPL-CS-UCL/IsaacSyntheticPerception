class RigidBody:
    """
    RigidBody objects that contains state information of the rigid body.
    """
    def __init__(self, prim, dc):
        """
        Initializes for RigidBody object
        
        Args:
            prim (pxr.Usd.Prim): rigid body prim
            dc (omni.isaac.motion_planning._motion_planning.MotionPlanning): motion planning interface from RMP extension
        """
        self.prim = prim
        self._dc = dc
        self.name = prim.GetPrimPath().name
        self.handle = self.get_rigid_body_handle()

    def get_rigid_body_handle(self):
        """
        Get rigid body handle.
        """
        object_children = self.prim.GetChildren()
        for child in object_children:
            child_path = child.GetPath().pathString
            body_handle = self._dc.get_rigid_body(child_path)
            if body_handle != 0:
                bin_path = child_path

        object_handle = self._dc.get_rigid_body(bin_path)
        if object_handle != 0: return object_handle

    def get_linear_velocity(self):
        """
        Get linear velocity of rigid body.
        """
        return np.array(self._dc.get_rigid_body_linear_velocity(self.handle))

    def get_angular_velocity(self):
        """
        Get angular velocity of rigid body.
        """
        return np.array(self._dc.get_rigid_body_angular_velocity(self.handle))

    def get_speed(self):
        """
        Get speed of rigid body given by the l2 norm of the velocity.
        """
        velocity = self.get_linear_velocity()
        speed = np.linalg.norm(velocity)
        return speed

    def get_pose(self):
        """
        Get pose of the rigid body containing the position and orientation information.
        """
        return self._dc.get_rigid_body_pose(self.handle)

    def get_position(self):
        """
        Get the position of the rigid body object.
        """
        pose = self.get_pose()
        position = np.array(pose.p)
        return position
    
    def get_orientation(self):
        """
        Get orientation of the rigid body object.
        """
        pose = self.get_pose()
        orientation = np.array(pose.r)
        return orientation

    def get_bound(self):
        """
        Get bounds of the rigid body object in global coordinates.
        """
        bound = UsdGeom.Mesh(self.prim).ComputeWorldBound(0.0, "default").GetBox()
        return [np.array(bound.GetMin()), np.array(bound.GetMax())]

    def __repr__(self):
        return self.name
