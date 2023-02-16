import numpy as np

from ..entities import Collision


class CollisionDetector:
    def __init__(self) -> None:
        self.collision_dict: dict[int, str] = {
            2: 'body',
            3: 'body',
            4: 'body',
            10: 'shoulder_left',
            18: 'upperarm_left',
            25: 'forearm_left',
            26: 'forearm_left',
            30: 'wrist_1_left',
            34: 'wrist_2_left',
            40: 'wrist_3_left',
            46: 'shoulder_right',
            54: 'upperarm_right',
            61: 'forearm_right',
            62: 'forearm_right',
            66: 'wrist_1_right',
            70: 'wrist_2_right',
            76: 'wrist_3_right',
        }

        self.ignore_list: list[tuple[str, str]] = [('shoulder_left', 'body'), ('shoulder_right', 'body')]

    def check_left_manipulator(self, geometry_1_id_list: np.ndarray, geometry_2_id_list: np.ndarray) -> Collision:
        if not geometry_1_id_list.size or not geometry_2_id_list.size:
            return Collision.empty()

        geometry_1_list: list[str] = [self.collision_dict.get(geometry_1_id, 'unknown') for geometry_1_id in geometry_1_id_list]
        geometry_2_list: list[str] = [self.collision_dict.get(geometry_2_id, 'unknown') for geometry_2_id in geometry_2_id_list]

        shoulder_collision_list: list[str] = []
        upperarm_collision_list: list[str] = []
        forearm_collision_list: list[str] = []
        wrist_1_collision_list: list[str] = []
        wrist_2_collision_list: list[str] = []
        wrist_3_collision_list: list[str] = []

        for geometry_1, geometry_2 in zip(geometry_1_list, geometry_2_list):
            if (geometry_1, geometry_2) in self.ignore_list:
                continue

            match geometry_1:
                case 'shoulder_left':
                    shoulder_collision_list.append(geometry_2) if geometry_2 not in shoulder_collision_list else None
                case 'upperarm_left':
                    upperarm_collision_list.append(geometry_2) if geometry_2 not in upperarm_collision_list else None
                case 'forearm_left':
                    forearm_collision_list.append(geometry_2) if geometry_2 not in forearm_collision_list else None
                case 'wrist_1_left':
                    wrist_1_collision_list.append(geometry_2) if geometry_2 not in wrist_1_collision_list else None
                case 'wrist_2_left':
                    wrist_2_collision_list.append(geometry_2) if geometry_2 not in wrist_2_collision_list else None
                case 'wrist_3_left':
                    wrist_3_collision_list.append(geometry_2) if geometry_2 not in wrist_3_collision_list else None

        collision_vector: list[bool] = [True if shoulder_collision_list else False,
                                        True if upperarm_collision_list else False,
                                        True if forearm_collision_list else False,
                                        True if wrist_1_collision_list else False,
                                        True if wrist_2_collision_list else False,
                                        True if wrist_3_collision_list else False,]

        return Collision(any(collision_vector),
                         collision_vector,
                         shoulder_collision_list,
                         upperarm_collision_list,
                         forearm_collision_list,
                         wrist_1_collision_list,
                         wrist_2_collision_list,
                         wrist_3_collision_list,
                         )

    def check_right_manipulator(self, geometry_1_id_list: np.ndarray, geometry_2_id_list: np.ndarray) -> Collision:
        if not geometry_1_id_list.size or not geometry_2_id_list.size:
            return Collision.empty()

        geometry_1_list: list[str] = [self.collision_dict.get(geometry_1_id, 'unknown') for geometry_1_id in geometry_1_id_list]
        geometry_2_list: list[str] = [self.collision_dict.get(geometry_2_id, 'unknown') for geometry_2_id in geometry_2_id_list]

        shoulder_collision_list: list[str] = []
        upperarm_collision_list: list[str] = []
        forearm_collision_list: list[str] = []
        wrist_1_collision_list: list[str] = []
        wrist_2_collision_list: list[str] = []
        wrist_3_collision_list: list[str] = []

        for geometry_1, geometry_2 in zip(geometry_1_list, geometry_2_list):
            if (geometry_1, geometry_2) in self.ignore_list:
                continue

            match geometry_1:
                case 'shoulder_right':
                    shoulder_collision_list.append(geometry_2) if geometry_2 not in shoulder_collision_list else None
                case 'upperarm_right':
                    upperarm_collision_list.append(geometry_2) if geometry_2 not in upperarm_collision_list else None
                case 'forearm_right':
                    forearm_collision_list.append(geometry_2) if geometry_2 not in forearm_collision_list else None
                case 'wrist_1_right':
                    wrist_1_collision_list.append(geometry_2) if geometry_2 not in wrist_1_collision_list else None
                case 'wrist_2_right':
                    wrist_2_collision_list.append(geometry_2) if geometry_2 not in wrist_2_collision_list else None
                case 'wrist_3_right':
                    wrist_3_collision_list.append(geometry_2) if geometry_2 not in wrist_3_collision_list else None

        collision_vector: list[bool] = [True if shoulder_collision_list else False,
                                        True if upperarm_collision_list else False,
                                        True if forearm_collision_list else False,
                                        True if wrist_1_collision_list else False,
                                        True if wrist_2_collision_list else False,
                                        True if wrist_3_collision_list else False,]

        return Collision(any(collision_vector),
                         collision_vector,
                         shoulder_collision_list,
                         upperarm_collision_list,
                         forearm_collision_list,
                         wrist_1_collision_list,
                         wrist_2_collision_list,
                         wrist_3_collision_list,
                         )
