from scipy.spatial.transform import Rotation

def euler2mat(ai,aj,ak,axes='sxyz'):
    if axes!='sxyz':
        raise ValueError("Pupper fixed xyz convention is required")
    return Rotation.from_euler('xyz',[ai,aj,ak]).as_matrix()
