import omni.usd
import omni.client
import omni.kit.commands
from pxr import UsdGeom, Sdf

def create_parent_xforms(asset_usd_path, save_as_path=None):
    omni.usd.get_context().open_stage(asset_usd_path)
    stage = omni.usd.get_context().get_stage()
    source_prim_path = stage.GetDefaultPrim().GetPath()

    prims = [stage.GetPrimAtPath(source_prim_path)]
    edits = Sdf.BatchNamespaceEdit()
    while len(prims) > 0:
        prim = prims.pop(0)

        if prim:
            if  prim.GetTypeName() in ["Mesh", "Capsule", "Sphere", "Box"]:
                    path = prim.GetRelationship('material:binding').GetTargets()[0]
                    mat_name= str(path).split("/")[-1]
                    new_path = str(prim.GetPath()) + "_xform/" + mat_name
                    omni.kit.commands.execute('MovePrim',
                    path_from=path,
                    path_to=new_path,
                    keep_world_transform=False,
                    destructive=False)
                    new_xform = UsdGeom.Xform.Define(stage, str(prim.GetPath()) + "_xform")
                    edits.Add(Sdf.NamespaceEdit.Reparent(prim.GetPath(), new_xform.GetPath(), 0)) 
                    continue
                    #print(prim.get_applied_visual_material())
            
            children_prims = prim.GetChildren()
            prims = prims + children_prims

    stage.GetRootLayer().Apply(edits)

    if save_as_path is None:
        omni.usd.get_context().save_stage()

    else:
        print(save_as_path)
        omni.usd.get_context().save_as_stage(save_as_path)
def convert_asset_instanceable(asset_usd_path, save_as_path=None, create_xforms=True):


    if create_xforms:
        create_parent_xforms(asset_usd_path, save_as_path)
        asset_usd_path = save_as_path

    instance_usd_path = ".".join(asset_usd_path.split(".")[:-1]) + "_meshes.usd"
    omni.client.copy(asset_usd_path, instance_usd_path)
    omni.usd.get_context().open_stage(asset_usd_path)
    stage = omni.usd.get_context().get_stage()
    source_prim_path = stage.GetDefaultPrim().GetPath()
    stage.GetDefaultPrim().GetReferences().AddReference(assetPath=instance_usd_path, primPath=source_prim_path)
    stage.GetDefaultPrim().SetInstanceable(True)
    prims = [stage.GetPrimAtPath(source_prim_path)]
    while len(prims) > 0:
        prim = prims.pop(0)
        if prim:
            if prim.GetTypeName() in ["Mesh", "Capsule", "Sphere", "Box"]:
                parent_prim = prim.GetParent()
                if parent_prim and not parent_prim.IsInstance():
                    parent_prim.GetReferences().AddReference(assetPath=instance_usd_path, primPath=str(parent_prim.GetPath()))
                    parent_prim.SetInstanceable(True)
                    continue

            children_prims = prim.GetChildren()
            prims = prims + children_prims

    if save_as_path is None:

        omni.usd.get_context().save_stage()
    else:
        print(save_as_path)
        omni.usd.get_context().save_as_stage(save_as_path)

convert_asset_instanceable("/home/jon/Documents/IsaacContent/ov-vegetation3dpack-01.100.1.0.linux-x86_64-ent-package/Trees/Gray_Birch_fall.usd",
save_as_path="/home/jon/Desktop/temp.usd")
print("done")
