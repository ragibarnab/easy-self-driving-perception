import numpy as np

def xy2theta(x, y):
    if (x >= 0 and y >= 0): 
        theta = 180/np.pi * np.arctan(y/x)
    elif (x < 0 and y >= 0): 
        theta = 180 - ((180/np.pi) * np.arctan(y/(-x)))
    elif (x < 0 and y < 0): 
        theta = 180 + ((180/np.pi) * np.arctan(y/x))
    elif ( x >= 0 and y < 0):
        theta = 360 - ((180/np.pi) * np.arctan((-y)/x))

    return theta


def point_to_ring_sector(point, gap_ring, gap_sector, num_ring, num_sector):
    x = point[0]
    y = point[1]
    # z = point[2]
    
    if(x == 0.0):
        x = 0.001
    if(y == 0.0):
        y = 0.001
    
    theta = xy2theta(x, y)
    faraway = np.sqrt(x*x + y*y)
    
    idx_ring = np.divmod(faraway, gap_ring)[0]       
    idx_sector = np.divmod(theta, gap_sector)[0]

    if(idx_ring >= num_ring):
        idx_ring = num_ring-1 # python starts with 0 and ends with N-1
    
    return int(idx_ring), int(idx_sector)