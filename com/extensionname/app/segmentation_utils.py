import numpy as np
from .states import FullState

def get_ND_bounding_box(volume, margin = None):
    """
    get the bounding box of nonzero region in an ND volume
    """
    input_shape = volume.shape
    if(margin is None):
        margin = [0] * len(input_shape)
    assert(len(input_shape) == len(margin))
    indxes = np.nonzero(volume)
    idx_min = []
    idx_max = []
    for i in range(len(input_shape)):
        idx_min.append(indxes[i].min())
        idx_max.append(indxes[i].max() + 1)

    for i in range(len(input_shape)):
        idx_min[i] = max(idx_min[i] - margin[i], 0)
        idx_max[i] = min(idx_max[i] + margin[i], input_shape[i])
    return idx_min, idx_max 


def get_instance_crop(state, object_id=2):
    """ Gets the instance crop of a given object.
    """
    if isinstance(state,dict):
        DeprecationWarning("State should not be a dictionary.")
        rgb = state["rgb"]
        seg = state["segmentation"]
        # cam_info = state["camera_info"]
        # sem_ids = np.unique(seg)
        # TODO: Missing mapping
        object_id = object_id #[s[0] for s in sem_list if s[3]=='object'] [0]

    elif isinstance(state, FullState):
        rgb = state.rgb
        # cam_info = state.camera_info
        if len(state.segm) == 2:
            seg = state.segm[0]
            sem_list = state.segm[1]
            # TODO: only accepts object.
            print("Not using object_id warning!!!")
            object_id = [s[0] for s in sem_list if s[3]=='object'] [0]
        else: 
            # Passed only instance segmentation without object mapping
            print("Not using object_id warning!!!")
            seg = state.segm
    else: raise ValueError(f"Unknown type {type(state)}")
    
    idx = get_ND_bounding_box(seg==object_id, margin=[15,15])
    mask_obj = (seg==object_id)[idx[0][0]:idx[1][0], idx[0][1]:idx[1][1]]
    rgb_obj = (rgb)[idx[0][0]:idx[1][0], idx[0][1]:idx[1][1],:3]
    metainfo = dict(bbox = idx, object_id=object_id)
    return rgb_obj, mask_obj, metainfo