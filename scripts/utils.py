import numpy as np

def dist_point_to_segment_signed(point, seg_start, seg_end):
    """
    Calcule la distance signée d'un point à un segment de droite.

    Parameters:
    point (tuple or list or numpy array): coordonnées (x, y) du point.
    seg_start (tuple or list or numpy array): coordonnées (x, y) du début du segment.
    seg_end (tuple or list or numpy array): coordonnées (x, y) de la fin du segment.

    Returns:
    float: distance signée du point au segment.
    """
    # Convertir les points en numpy arrays pour faciliter les calculs
    point = np.array(point)
    seg_start = np.array(seg_start)
    seg_end = np.array(seg_end)
    
    # Vecteurs du segment et du point au début du segment
    segment_vector = seg_end - seg_start
    point_vector = point - seg_start
    
    # Longueur du segment
    segment_length = np.linalg.norm(segment_vector)
    
    if segment_length == 0:
        # Le segment est en fait un point
        return np.linalg.norm(point_vector)
    
    # Vecteur unitaire du segment
    segment_unit_vector = segment_vector / segment_length
    
    # Projeter le vecteur point_vector sur le vecteur unitaire du segment
    projection_length = np.dot(point_vector, segment_unit_vector)
    
    # Calculer la projection sur le segment
    if projection_length < 0:
        # Le point est avant seg_start
        closest_point_on_segment = seg_start
    elif projection_length > segment_length:
        # Le point est après seg_end
        closest_point_on_segment = seg_end
    else:
        # Le point est quelque part entre seg_start et seg_end
        closest_point_on_segment = seg_start + projection_length * segment_unit_vector
    
    # Distance euclidienne du point au point le plus proche sur le segment
    distance_to_segment = np.linalg.norm(point - closest_point_on_segment)
    
    # Calculer la direction signée
    cross_product = np.cross(segment_vector, point - seg_start)
    
    if cross_product < 0:
        return -distance_to_segment  # Le point est à droite du segment
    else:
        return distance_to_segment  # Le point est à gauche du segment ou sur le segment

# test
if __name__ == "__main__":
    point = (3, -1)
    seg_start = (0, 0)
    seg_end = (5, 0)
    distance = dist_point_to_segment_signed(point, seg_start, seg_end)
    print(f"La distance signée du point au segment est: {distance}")
        