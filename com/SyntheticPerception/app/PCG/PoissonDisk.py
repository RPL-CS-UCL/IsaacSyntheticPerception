import numpy as np
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('TkAgg')

def get_cell_coords(pt, a):
    """Get the coordinates of the cell that pt = (x,y) falls in."""

    return int(pt[0] // a), int(pt[1] // a)

def get_neighbours(coords,cells, nx, ny):
    """Return the indexes of points in cells neighbouring cell at coords.

    For the cell at coords = (x,y), return the indexes of points in the cells
    with neighbouring coordinates illustrated below: ie those cells that could 
    contain points closer than r.

                                     ooo
                                    ooooo
                                    ooXoo
                                    ooooo
                                     ooo

    """

    dxdy = [(-1,-2),(0,-2),(1,-2),(-2,-1),(-1,-1),(0,-1),(1,-1),(2,-1),
            (-2,0),(-1,0),(1,0),(2,0),(-2,1),(-1,1),(0,1),(1,1),(2,1),
            (-1,2),(0,2),(1,2),(0,0)]
    neighbours = []
    for dx, dy in dxdy:
        neighbour_coords = coords[0] + dx, coords[1] + dy
        if not (0 <= neighbour_coords[0] < nx and
                0 <= neighbour_coords[1] < ny):
            # We're off the grid: no neighbours here.
            continue
        neighbour_cell = cells[neighbour_coords]
        if neighbour_cell is not None:
            # This cell is occupied: store this index of the contained point.
            neighbours.append(neighbour_cell)
    return neighbours

def point_valid(pt):
    """Is pt a valid point to emit as a sample?

    It must be no closer than r from any other point: check the cells in its
    immediate neighbourhood.

    """

    cell_coords = get_cell_coords(pt)
    for idx in get_neighbours(cell_coords, cells):
        nearby_pt = samples[idx]
        # Squared distance between or candidate point, pt, and this nearby_pt.
        distance2 = (nearby_pt[0]-pt[0])**2 + (nearby_pt[1]-pt[1])**2
        if distance2 < r**2:
            # The points are too close, so pt is not a candidate.
            return False
    # All points tested: if we're here, pt is valid
    return True

def get_point(k, refpt):
    """Try to find a candidate point relative to refpt to emit in the sample.

    We draw up to k points from the annulus of inner radius r, outer radius 2r
    around the reference point, refpt. If none of them are suitable (because
    they're too close to existing points in the sample), return False.
    Otherwise, return the pt.

    """
    i = 0
    while i < k:
        i += 1
        rho = np.sqrt(np.random.uniform(r**2, 4 * r**2))
        theta = np.random.uniform(0, 2*np.pi)
        pt = refpt[0] + rho*np.cos(theta), refpt[1] + rho*np.sin(theta)
        if not (0 <= pt[0] < width and 0 <= pt[1] < height):
            # This point falls outside the domain, so try again.
            continue
        if point_valid(pt):
            return pt

    # We failed to find a suitable point in the vicinity of refpt.
    return False

def generate_points(width, height, min_dist, max_sample_points):
# Choose up to k points around each reference point as candidates for a new
# sample point
    k = max_sample_points

# Minimum distance between samples
    r = min_dist

    width, height = 60, 45

# Cell side length
    ar = r/np.sqrt(2)
# Number of cells in the x- and y-directions of the grid
    nx, ny = int(width / ar) + 1, int(height / ar) + 1

# A list of coordinates in the grid of cells
    coords_list = [(ix, iy) for ix in range(nx) for iy in range(ny)]
# Initilalize the dictionary of cells: each key is a cell's coordinates, the
# corresponding value is the index of that cell's point's coordinates in the
# samples list (or None if the cell is empty).
    cells = {coords: None for coords in coords_list}

# Pick a random point to start with.
    pt = (np.random.uniform(0, width), np.random.uniform(0, height))
    samples = [pt]
# Our first sample is indexed at 0 in the samples list...
    cells[get_cell_coords(pt, ar)] = 0
# ... and it is active, in the sense that we're going to look for more points
# in its neighbourhood.
    active = [0]

    nsamples = 1
# As long as there are points in the active list, keep trying to find samples.
    while active:
        # choose a random "reference" point from the active list.
        idx = np.random.choice(active)
        refpt = samples[idx]
        # Try to pick a new point relative to the reference point.
        pt = get_point(k, refpt)
        if pt:
            # Point pt is valid: add it to the samples list and mark it as active
            samples.append(pt)
            nsamples += 1
            active.append(len(samples)-1)
            cells[get_cell_coords(pt)] = len(samples) - 1
        else:
            # We had to give up looking for valid points near refpt, so remove it
            # from the list of "active" points.
            active.remove(idx)

    plt.scatter(*zip(*samples), color='r', alpha=0.6, lw=0)
    plt.xlim(0, width)
    plt.ylim(0, height)
    plt.axis('off')
    plt.show()
def Bridson_sampling(width=1.0, height=1.0, radius=0.025, k=30):
    # References: Fast Poisson Disk Sampling in Arbitrary Dimensions
    #             Robert Bridson, SIGGRAPH, 2007
    def squared_distance(p0, p1):
        return (p0[0]-p1[0])**2 + (p0[1]-p1[1])**2

    def random_point_around(p, k=1):
        # WARNING: This is not uniform around p but we can live with it
        R = np.random.uniform(radius, 2*radius, k)
        T = np.random.uniform(0, 2*np.pi, k)
        P = np.empty((k, 2))
        P[:, 0] = p[0]+R*np.sin(T)
        P[:, 1] = p[1]+R*np.cos(T)
        return P

    def in_limits(p):
        return 0 <= p[0] < width and 0 <= p[1] < height

    def neighborhood(shape, index, n=2):
        row, col = index
        row0, row1 = max(row-n, 0), min(row+n+1, shape[0])
        col0, col1 = max(col-n, 0), min(col+n+1, shape[1])
        I = np.dstack(np.mgrid[row0:row1, col0:col1])
        I = I.reshape(I.size//2, 2).tolist()
        I.remove([row, col])
        return I

    def in_neighborhood(p):
        i, j = int(p[0]/cellsize), int(p[1]/cellsize)
        if M[i, j]:
            return True
        for (i, j) in N[(i, j)]:
            if M[i, j] and squared_distance(p, P[i, j]) < squared_radius:
                return True
        return False

    def add_point(p):
        points.append(p)
        i, j = int(p[0]/cellsize), int(p[1]/cellsize)
        P[i, j], M[i, j] = p, True

    # Here `2` corresponds to the number of dimension
    cellsize = radius/np.sqrt(2)
    rows = int(np.ceil(width/cellsize))
    cols = int(np.ceil(height/cellsize))

    # Squared radius because we'll compare squared distance
    squared_radius = radius*radius

    # Positions cells
    P = np.zeros((rows, cols, 2), dtype=np.float32)
    M = np.zeros((rows, cols), dtype=bool)

    # Cache generation for neighborhood
    N = {}
    for i in range(rows):
        for j in range(cols):
            N[(i, j)] = neighborhood(M.shape, (i, j), 2)

    points = []
    add_point((np.random.uniform(width), np.random.uniform(height)))
    while len(points):
        i = np.random.randint(len(points))
        p = points[i]
        del points[i]
        Q = random_point_around(p, k)
        for q in Q:
            if in_limits(q) and not in_neighborhood(q):
                add_point(q)
    points = P[M]
    return points


if __name__ == '__main__':

    plt.figure()
    plt.subplot(1, 1, 1, aspect=1)

    points = Bridson_sampling()
    X = [x for (x, y) in points]
    Y = [y for (x, y) in points]
    plt.scatter(X, Y, s=10)
    plt.xlim(0, 1)
    plt.ylim(0, 1)
    plt.show()
