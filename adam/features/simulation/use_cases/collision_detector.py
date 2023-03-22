import numpy as np

from ..entities import Collision


class CollisionDetector:
    def __init__(self) -> None:
        self.self_collision_dict: dict[int, str] = {
            2: 'body',
            8: 'shoulder_left',
            16: 'upperarm_left',
            23: 'forearm_left',
            24: 'forearm_left',
            28: 'wrist_1_left',
            32: 'wrist_2_left',
            38: 'wrist_3_left',
            44: 'shoulder_right',
            52: 'upperarm_right',
            59: 'forearm_right',
            60: 'forearm_right',
            64: 'wrist_1_right',
            68: 'wrist_2_right',
            74: 'wrist_3_right',
        }

        self.env_collision_dict: dict[int, str] = {}
        self.global_collision_dict: dict[int, str] = self.self_collision_dict.copy()

        self.ignore_list: list[tuple[str, str]] = [('shoulder_left', 'body'), ('shoulder_right', 'body')]

    def extend_collisions(self, collision_dict: dict[int, str]) -> None:
        self.env_collision_dict.update(collision_dict)
        self.global_collision_dict.update(collision_dict)

    def check_left_manipulator(self, geometry_1_id_list: np.ndarray, geometry_2_id_list: np.ndarray) -> Collision:
        # Return if lists are empty
        if not geometry_1_id_list.size or not geometry_2_id_list.size:
            return Collision.empty()

        # Map IDs to components
        geometry_1_list: list[str] = [self.global_collision_dict.get(geometry_1_id, 'unknown') for geometry_1_id in geometry_1_id_list]
        geometry_2_list: list[str] = [self.global_collision_dict.get(geometry_2_id, 'unknown') for geometry_2_id in geometry_2_id_list]

        shoulder_collision_list: list[str] = []
        upperarm_collision_list: list[str] = []
        forearm_collision_list: list[str] = []
        wrist_1_collision_list: list[str] = []
        wrist_2_collision_list: list[str] = []
        wrist_3_collision_list: list[str] = []

        self_collision: bool = False
        env_collision: bool = False

        for geometry_1, geometry_2 in zip(geometry_1_list, geometry_2_list):
            if (geometry_1, geometry_2) in self.ignore_list:
                continue

            match geometry_1:
                case 'shoulder_left':
                    shoulder_collision_list.append(geometry_2) if geometry_2 not in shoulder_collision_list else None
                    self_collision = True if geometry_2 in self.self_collision_dict.values() else False
                    env_collision = True if geometry_2 in self.env_collision_dict.values() or geometry_2 == 'unknown' else False
                case 'upperarm_left':
                    upperarm_collision_list.append(geometry_2) if geometry_2 not in upperarm_collision_list else None
                    self_collision = True if geometry_2 in self.self_collision_dict.values() else False
                    env_collision = True if geometry_2 in self.env_collision_dict.values() or geometry_2 == 'unknown' else False
                case 'forearm_left':
                    forearm_collision_list.append(geometry_2) if geometry_2 not in forearm_collision_list else None
                    self_collision = True if geometry_2 in self.self_collision_dict.values() else False
                    env_collision = True if geometry_2 in self.env_collision_dict.values() or geometry_2 == 'unknown' else False
                case 'wrist_1_left':
                    wrist_1_collision_list.append(geometry_2) if geometry_2 not in wrist_1_collision_list else None
                    self_collision = True if geometry_2 in self.self_collision_dict.values() else False
                    env_collision = True if geometry_2 in self.env_collision_dict.values() or geometry_2 == 'unknown' else False
                case 'wrist_2_left':
                    wrist_2_collision_list.append(geometry_2) if geometry_2 not in wrist_2_collision_list else None
                    self_collision = True if geometry_2 in self.self_collision_dict.values() else False
                    env_collision = True if geometry_2 in self.env_collision_dict.values() or geometry_2 == 'unknown' else False
                case 'wrist_3_left':
                    wrist_3_collision_list.append(geometry_2) if geometry_2 not in wrist_3_collision_list else None
                    self_collision = True if geometry_2 in self.self_collision_dict.values() else False
                    env_collision = True if geometry_2 in self.env_collision_dict.values() or geometry_2 == 'unknown' else False

        collision_vector: list[bool] = [True if shoulder_collision_list else False,
                                        True if upperarm_collision_list else False,
                                        True if forearm_collision_list else False,
                                        True if wrist_1_collision_list else False,
                                        True if wrist_2_collision_list else False,
                                        True if wrist_3_collision_list else False,]

        return Collision(collided=any(collision_vector),
                         self_collision=self_collision,
                         env_collision=env_collision,
                         vector=collision_vector,
                         shoulder=shoulder_collision_list,
                         upper_arm=upperarm_collision_list,
                         forearm=forearm_collision_list,
                         wrist_1=wrist_1_collision_list,
                         wrist_2=wrist_2_collision_list,
                         wrist_3=wrist_3_collision_list,
                         )

    def check_right_manipulator(self, geometry_1_id_list: np.ndarray, geometry_2_id_list: np.ndarray) -> Collision:
        if not geometry_1_id_list.size or not geometry_2_id_list.size:
            return Collision.empty()

        geometry_1_list: list[str] = [self.global_collision_dict.get(geometry_1_id, 'unknown') for geometry_1_id in geometry_1_id_list]
        geometry_2_list: list[str] = [self.global_collision_dict.get(geometry_2_id, 'unknown') for geometry_2_id in geometry_2_id_list]

        shoulder_collision_list: list[str] = []
        upperarm_collision_list: list[str] = []
        forearm_collision_list: list[str] = []
        wrist_1_collision_list: list[str] = []
        wrist_2_collision_list: list[str] = []
        wrist_3_collision_list: list[str] = []

        self_collision: bool = False
        env_collision: bool = False

        for geometry_1, geometry_2 in zip(geometry_1_list, geometry_2_list):
            if (geometry_1, geometry_2) in self.ignore_list:
                continue

            match geometry_1:
                case 'shoulder_right':
                    shoulder_collision_list.append(geometry_2) if geometry_2 not in shoulder_collision_list else None
                    self_collision = True if geometry_2 in self.self_collision_dict.values() else False
                    env_collision = True if geometry_2 in self.env_collision_dict.values() or geometry_2 == 'unknown' else False
                case 'upperarm_right':
                    upperarm_collision_list.append(geometry_2) if geometry_2 not in upperarm_collision_list else None
                    self_collision = True if geometry_2 in self.self_collision_dict.values() else False
                    env_collision = True if geometry_2 in self.env_collision_dict.values() or geometry_2 == 'unknown' else False
                case 'forearm_right':
                    forearm_collision_list.append(geometry_2) if geometry_2 not in forearm_collision_list else None
                    self_collision = True if geometry_2 in self.self_collision_dict.values() else False
                    env_collision = True if geometry_2 in self.env_collision_dict.values() or geometry_2 == 'unknown' else False
                case 'wrist_1_right':
                    wrist_1_collision_list.append(geometry_2) if geometry_2 not in wrist_1_collision_list else None
                    self_collision = True if geometry_2 in self.self_collision_dict.values() else False
                    env_collision = True if geometry_2 in self.env_collision_dict.values() or geometry_2 == 'unknown' else False
                case 'wrist_2_right':
                    wrist_2_collision_list.append(geometry_2) if geometry_2 not in wrist_2_collision_list else None
                    self_collision = True if geometry_2 in self.self_collision_dict.values() else False
                    env_collision = True if geometry_2 in self.env_collision_dict.values() or geometry_2 == 'unknown' else False
                case 'wrist_3_right':
                    wrist_3_collision_list.append(geometry_2) if geometry_2 not in wrist_3_collision_list else None
                    self_collision = True if geometry_2 in self.self_collision_dict.values() else False
                    env_collision = True if geometry_2 in self.env_collision_dict.values() or geometry_2 == 'unknown' else False

        collision_vector: list[bool] = [True if shoulder_collision_list else False,
                                        True if upperarm_collision_list else False,
                                        True if forearm_collision_list else False,
                                        True if wrist_1_collision_list else False,
                                        True if wrist_2_collision_list else False,
                                        True if wrist_3_collision_list else False,]

        return Collision(collided=any(collision_vector),
                         self_collision=self_collision,
                         env_collision=env_collision,
                         vector=collision_vector,
                         shoulder=shoulder_collision_list,
                         upper_arm=upperarm_collision_list,
                         forearm=forearm_collision_list,
                         wrist_1=wrist_1_collision_list,
                         wrist_2=wrist_2_collision_list,
                         wrist_3=wrist_3_collision_list,
                         )
