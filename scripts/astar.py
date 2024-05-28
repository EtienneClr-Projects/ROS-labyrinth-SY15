from math import sqrt
import numpy as np
import queue

class Etats():
    def __init__(self, x, y, cost, liste_actions):
        self.x = x
        self.y = y
        self.cost = cost
        self.liste_actions = liste_actions

    def __eq__(self, other):
        if hash(self) == hash(other):
            return True
        return False
    
    def __hash__(self):
        return hash((self.x, self.y))
    
    def __lt__(self, other):
        return self.cost < other.cost
    
    def __str__(self):
        return self.liste_actions

# distance euclidienne entre 2 points
def dist(pos1, pos2):
    return np.linalg.norm(np.array(pos2) - np.array(pos1))

# calcul de la distance par rapport au mur le plus proche
# on va chercher à s'en éloigner + recherche dans un rayon de 2 cases autours de la position cible
def distance_to_nearest_wall(pos, costmap, costmap_size):
    x, y = pos
    if costmap[y, x] >= 100:
        return 0
    distances = []
    for i in range(max(0, x-2), min(costmap_size, x+3)):
        for j in range(max(0, y-2), min(costmap_size, y+3)):
            if costmap[j, i] >= 100:
                distances.append(dist((x, y), (i, j)))
    return min(distances) if distances else float('inf')


def convert_sequence_to_poses_path(path, start,resolution, origin_x, origin_y):
    """
    path is a sequence of (x,y) movement, with x,y = -1, 0 or 1
    we convert the sequence to have only the keypoints of the movements
    for instance the sequence (1,0) (1,0) (1,0) (0,1) will be (3,0) (0,1)
    and then we convert the keypoints to global positions relative to start
    """
    path.append((0,0))
    path = np.array(path)
    last_move = np.array([0,0])
    sum_of_last_moves = np.array([0,0])
    new_sequence = []
    for move in path:
        if np.all(move == last_move):
            sum_of_last_moves += move
            last_move = move
        else:
            last_move = move
            new_sequence.append(sum_of_last_moves)
            sum_of_last_moves = move

    # now we convert each movement as a global position
    current = start
    poses_path = []
    for move in new_sequence:
        current = [
            move[0] * resolution + origin_x + current[0],
            move[1] * resolution + origin_y + current[1]
        ]
        poses_path.append(current)

    return poses_path


def astar(start, goal, costmap_size, resolution, origin_x, origin_y, costmap, last_path_cost, last_path):
    grid_start = (
        int((start[0] - origin_x) / resolution + costmap_size // 2),
        int((start[1] - origin_y) / resolution + costmap_size // 2)
    )

    grid_goal = (
        int((goal[0] - origin_x) / resolution + costmap_size // 2),
        int((goal[1] - origin_y) / resolution + costmap_size // 2)
    )

    state = Etats(grid_start[0], grid_start[1], 0, [])

    # liste des actions possible pour le robot
    actions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    # actions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (1, 1), (-1, 1), (1, -1)] # avec diagonales
    # setup des listes
    liste_chemin = queue.PriorityQueue()
    chemin_deja_visite = set()
    # première valeur dans la liste
    liste_chemin.put((0, state))
    while not liste_chemin.empty():
        # on récupère le tuple en première position dans la liste
        node = liste_chemin.get()
        current_state = node[1]
        # on vérifie si c'est un chemin déjà visité
        if current_state in chemin_deja_visite:
            continue
        chemin_deja_visite.add(current_state)
        for dx, dy in actions:
            next_pos = (current_state.x + dx, current_state.y + dy)
            # calcul du prochain coût pour la prochaine position
            next_cost = current_state.cost + 1
            # création du prochain état
            new_state = Etats(next_pos[0], next_pos[1], next_cost, current_state.liste_actions + [(dx, dy)])

            # on vérifie si la prochaine position est dans la grille / costmap : sécurité
            if not (0 <= next_pos[0] < costmap_size and 0 <= next_pos[1] < costmap_size):
                continue

            # on vérifie si la position cible est un mur
            if costmap[next_pos[1], next_pos[0]] >= 100:
                chemin_deja_visite.add(new_state)
                continue
            margin = 3  # marge de sécurité en nombre de cellules
            distance_wall = distance_to_nearest_wall(next_pos, costmap, costmap_size)

            # si on est trop proche d'un mur, la position ciblée prend un coup supplémentaire
            if distance_wall < margin:
                next_cost += (margin - distance_wall) * 10

            # on va chercher à éviter les changements de direction trop souvent
            if current_state.liste_actions:
                last_action = current_state.liste_actions[-1]
                # on punie les cas où on doit effectuer un changement de direction par rapport à l'action précédente
                # tourner, même en diagonal, a un coût
                if (last_action[0], last_action[1]) != (dx, dy):
                    next_cost += 2

            # on vérifie si c'est un chemin déjà visité
            if new_state in chemin_deja_visite:
                continue

            # on vérifie si c'est notre destination ou si on a atteint le nb d'actions max
            max_actions = 50
            if next_pos == grid_goal or len(new_state.liste_actions) >= max_actions:
                # if current cost is not different from last_path_cost, then we just keep last path to not change everytime
                if last_path_cost is not None and abs(next_cost - last_path_cost) < 10:
                    print("return same path, diff = ", abs(next_cost - last_path_cost))
                    return last_path, last_path_cost
                if last_path_cost is not None:
                    print(next_cost, last_path_cost, abs(next_cost - last_path_cost))
                poses_path = convert_sequence_to_poses_path(new_state.liste_actions, start, resolution, origin_x, origin_y)
                return poses_path, next_cost
            
            # on actualise le cost du prochain état en fonction des obstacles sur son chemin (distance mur, changement de direction par exemple)
            new_state.cost = next_cost
            
            # si aucune des conditions précédentes n'est remplie, on l'ajoute à notre liste des chemins à "étudier"
            liste_chemin.put((next_cost + dist(next_pos, grid_goal), new_state))
    return None, None
