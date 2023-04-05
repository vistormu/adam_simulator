import numpy as np

from .....entities import Collision


def _get_collision_element(id: int) -> str:
    if id >= 0 and id <= 3:
        return 'body'
    elif id == 4:
        return 'base'
    elif id >= 5 and id <= 9:
        return 'left_shoulder'
    elif id >= 10 and id <= 16:
        return 'left_upper_arm'
    elif id >= 17 and id <= 22:
        return 'left_forearm'
    elif id >= 23 and id <= 25:
        return 'left_wrist_1'
    elif id >= 26 and id <= 28:
        return 'left_wrist_2'
    elif id >= 29 and id <= 31:
        return 'left_wrist_3'
    elif id >= 32 and id <= 33:
        return 'left_wrist_3'
    elif id >= 34 and id <= 38:
        return 'right_shoulder'
    elif id >= 39 and id <= 45:
        return 'right_upper_arm'
    elif id >= 46 and id <= 51:
        return 'right_forearm'
    elif id >= 52 and id <= 54:
        return 'right_wrist_1'
    elif id >= 55 and id <= 57:
        return 'right_wrist_2'
    elif id >= 58 and id <= 60:
        return 'right_wrist_3'
    elif id >= 61 and id <= 62:
        return 'right_wrist_3'
    else:
        return 'unknown'


class CollisionChecker:
    @ staticmethod
    def get(geometry_1_id_list: np.ndarray, geometry_2_id_list: np.ndarray) -> Collision:
        if not geometry_1_id_list.size or not geometry_2_id_list.size:
            return Collision.empty()

        # Change id to element
        geometry_1_list: list[str] = [_get_collision_element(id) for id in geometry_1_id_list]
        geometry_2_list: list[str] = [_get_collision_element(id) for id in geometry_2_id_list]

        # Create collision pair list
        collision_pair_list: np.ndarray = np.array([geometry_1_list, geometry_2_list]).T
        collision_pair_list = np.concatenate((collision_pair_list, np.flip(collision_pair_list, axis=1)), axis=0)

        # Delete the row if a pair is unknown and if the other starts with 'right'
        for geometry_1, geometry_2 in collision_pair_list:
            if geometry_1 == 'unknown' and geometry_2.startswith('left'):
                collision_pair_list = np.delete(collision_pair_list, np.where((collision_pair_list == [geometry_1, geometry_2]).all(axis=1)), axis=0)
            elif geometry_2 == 'unknown' and geometry_1.startswith('left'):
                collision_pair_list = np.delete(collision_pair_list, np.where((collision_pair_list == [geometry_1, geometry_2]).all(axis=1)), axis=0)

        env_collision = True if np.any(collision_pair_list == 'unknown') else False
        collision_pair_list = collision_pair_list[~np.any(collision_pair_list == 'unknown', axis=1)]
        self_collision = True if collision_pair_list.size else False

        shoulder_collision_list = collision_pair_list[collision_pair_list[:, 0] == 'right_shoulder', 1].tolist()
        upper_arm_collision_list = collision_pair_list[collision_pair_list[:, 0] == 'right_upper_arm', 1].tolist()
        forearm_collision_list = collision_pair_list[collision_pair_list[:, 0] == 'right_forearm', 1].tolist()
        wrist_1_collision_list = collision_pair_list[collision_pair_list[:, 0] == 'right_wrist_1', 1].tolist()
        wrist_2_collision_list = collision_pair_list[collision_pair_list[:, 0] == 'right_wrist_2', 1].tolist()
        wrist_3_collision_list = collision_pair_list[collision_pair_list[:, 0] == 'right_wrist_3', 1].tolist()

        collision_vector: list[bool] = [True if shoulder_collision_list else False,
                                        True if upper_arm_collision_list else False,
                                        True if forearm_collision_list else False,
                                        True if wrist_1_collision_list else False,
                                        True if wrist_2_collision_list else False,
                                        True if wrist_3_collision_list else False,]

        return Collision(collided=self_collision or env_collision,
                         self_collision=self_collision,
                         env_collision=env_collision,
                         vector=collision_vector,
                         shoulder=shoulder_collision_list,
                         upper_arm=upper_arm_collision_list,
                         forearm=forearm_collision_list,
                         wrist_1=wrist_1_collision_list,
                         wrist_2=wrist_2_collision_list,
                         wrist_3=wrist_3_collision_list,
                         )
