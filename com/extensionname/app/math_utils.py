import numpy as np
import torch 
import pytorch_metric_learning.distances as distances

def normalize_vector(abc):
    if type(abc)==np.ndarray:
        factor=np.sqrt(np.dot(abc,abc)) 
        norm_abc =np.divide(abc,factor)
    elif torch.is_tensor(abc):
        factor=torch.sqrt(torch.dot(abc,abc)) 
        norm_abc =torch.div(abc,factor)
    return norm_abc

def get_normal(P,Q,R):
    PR=R-P
    PQ=Q-P
    
    if type(P)==np.ndarray:
        #Find normal vector (of which coefficients are a, b and c)
        PQ_skew=np.array([[0, -PQ[2],PQ[1]],
                         [PQ[2], 0, -PQ[0]],
                         [-PQ[1], PQ[0], 0]])
        abc=np.dot(PQ_skew,PR)
        norm_abc = normalize_vector(abc)
        return norm_abc
    elif torch.is_tensor(P):
        #Find normal vector (of which coefficients are a, b and c)
        PQ_skew=torch.Tensor([[0, -PQ[2],PQ[1]],
                         [PQ[2], 0, -PQ[0]],
                         [-PQ[1], PQ[0], 0]])
        abc=torch.matmul(PQ_skew,PR.unsqueeze(0).T)[:,0]
        
        norm_abc = normalize_vector(abc)
        return norm_abc
    raise NotImplementedError("Invalid type passed to get_normal")

    
def get_closest_point_indices(XYZ_t, interaction_pt, threshold=0.04):
    """
    Given a set of points XYZ_t, the function returns a 
    mask of all points which are within threshold distance
    from the interaction_pt
    : param XYZ_t : torch.Tensor, Nx3
    : interaction_pt : array-like, 1x3
    : threshold : float
    """
    from pytorch_metric_learning import distances
    from fast_pytorch_kmeans import KMeans
    # Split points into clusters
    kmeans = KMeans(n_clusters=40, mode='euclidean', verbose=1)
    labels = kmeans.fit_predict(XYZ_t)
    centroids = kmeans.centroids

    # Select query location(s) for interaction
    interaction_pt = torch.Tensor(interaction_pt).unsqueeze(0)

    # Get distances to the location(s)
    d_fn = distances.LpDistance(normalize_embeddings=False)
    D = d_fn(centroids,interaction_pt)

    # Get the labels of centroids which are selected as affordance area
    label_sel = torch.where(D[:,0]<threshold)[0]

    # make a mask of the object that defines its affordance area
    sel_mask_pts_bool = torch.isin(labels,label_sel)
    return sel_mask_pts_bool

def cosine_distance(A,B):
    d_cos_sim_fn = distances.CosineSimilarity()
    return d_cos_sim_fn(A,B)


### Get one-one correspondences
def get_corresponding_points(ft_q, ft_t):
    # find best match
    d_fn = distances.CosineSimilarity()
    D = d_fn(ft_q, ft_t) # D1xD2

    # Dsm = torch.nn.functional.softmax(D*15,dim=1)
    # Dsm*D

    Dargmax_QT = D.argmax(dim=1) # for each in 1, argmax in dimension 2
    Dargmax_TQ = D.argmax(dim=0) # and vice versa
    # Select the points A for which the closest matches B in T 
    # have closest matches A in Q
    init_pts_Q = torch.arange(D.shape[0]) 
    one_one_corr_Q = torch.where(Dargmax_TQ[Dargmax_QT] == init_pts_Q)[0] 
    #one_one_corr_Q = Dargmax_TQ[Dargmax_QT] == init_pts_Q
    init_pts_T = torch.arange(D.shape[1]) 
    #one_one_corr_T = init_pts_T[Dargmax_QT[one_one_corr_Q]]
    one_one_corr_T = Dargmax_QT[one_one_corr_Q]
    # map to image shape
    flag_mask = D[one_one_corr_Q, one_one_corr_T]>D[one_one_corr_Q, one_one_corr_T].mean()
    one_one_corr_T = one_one_corr_T[flag_mask]
    one_one_corr_Q = one_one_corr_Q[flag_mask]
    
    return one_one_corr_Q, one_one_corr_T, D[one_one_corr_Q, one_one_corr_T]
def get_corresponding_points_one_way(ft_q, ft_t):
    # find best match
    d_fn = distances.CosineSimilarity()
    D = d_fn(ft_q, ft_t) # D1xD2

    Dargmax_QT = D.argmax(dim=1) # for each in 1, argmax in dimension 2
    Dargmax_TQ = D.argmax(dim=0) # and vice versa
    # Select the points A for which the closest matches B in T 
    # have closest matches A in Q
    init_pts_Q = torch.arange(D.shape[0]) 
    flag_mask = D[init_pts_Q, Dargmax_QT]>D[init_pts_Q, Dargmax_QT].mean()
    one_one_corr_T = one_one_corr_T[flag_mask]
    one_one_corr_Q = one_one_corr_Q[flag_mask]
    
    return one_one_corr_Q, one_one_corr_T, D[one_one_corr_Q, one_one_corr_T]


    one_one_corr_Q = torch.where(Dargmax_TQ[Dargmax_QT] == init_pts_Q)[0] 
    #one_one_corr_Q = Dargmax_TQ[Dargmax_QT] == init_pts_Q
    init_pts_T = torch.arange(D.shape[1]) 
    #one_one_corr_T = init_pts_T[Dargmax_QT[one_one_corr_Q]]
    one_one_corr_T = Dargmax_QT[one_one_corr_Q]
    # map to image shape
    flag_mask = D[one_one_corr_Q, one_one_corr_T]>D[one_one_corr_Q, one_one_corr_T].mean()
    one_one_corr_T = one_one_corr_T[flag_mask]
    one_one_corr_Q = one_one_corr_Q[flag_mask]
    
    return one_one_corr_Q, one_one_corr_T, D[one_one_corr_Q, one_one_corr_T]


def torch_L2(x, dim=1):
    """L2 of 2D X"""
    try:
        return torch.sqrt(torch.sum(torch.pow(x,2),dim=dim))
    except:
        print ("Error in torch L2 norm, perhaps the shape ",x.shape," does not have dim ",dim)

def numpy_L2(x, dim=1):
    """L2 of 2D X"""
    return np.sqrt(np.sum(np.power(x,2),axis=dim))
def L2(x,dim=1):
    if torch.is_tensor(x):
        return torch_L2(x, dim)
    else: 
        return numpy_L2(x, dim)

def get_mask_locations(mask):
    if torch.is_tensor(mask):
        return torch.stack(torch.where(mask)).T
    else:
        return np.stack(np.where(mask)).T


def axis_angle_to_quat(axis_param, angle):
    qx = axis_param[0] * torch.sin(angle/2)
    qy = axis_param[1] * torch.sin(angle/2)
    qz = axis_param[2] * torch.sin(angle/2)
    qw = torch.cos(angle/2)
    return qx,qy,qz,qw

def quat_to_matrix(qx,qy,qz,qw):
    opt_matrix = torch.eye(4)

    opt_matrix[0, 0] = 1. - 2. * qy ** 2 - 2. * qz ** 2
    opt_matrix[1, 1] = 1. - 2. * qx ** 2 - 2. * qz ** 2
    opt_matrix[2, 2] = 1. - 2. * qx ** 2 - 2. * qy ** 2

    opt_matrix[0, 1] = 2. * qx * qy - 2. * qz * qw
    opt_matrix[1, 0] = 2. * qx * qy + 2. * qz * qw

    opt_matrix[0, 2] = 2. * qx * qz + 2 * qy * qw
    opt_matrix[2, 0] = 2. * qx * qz - 2 * qy * qw

    opt_matrix[1, 2] = 2. * qy * qz - 2. * qx * qw
    opt_matrix[2, 1] = 2. * qy * qz + 2. * qx * qw
    return opt_matrix