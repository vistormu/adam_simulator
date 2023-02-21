from dataclasses import dataclass


@dataclass
class Body:
    # Core
    name: str = 'body'
    type: str = 'type'

    # Geom
    position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    orientation: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    size: float | tuple[float, float] | tuple[float, float, float] = 0.0
    center_location: str = 'base_center'

    # Dynamics
    mass: float = 0.0
    center_of_mass: tuple[float, float, float] = (0.0, 0.0, 0.0)

    # Appearance
    color: str | tuple[float, float, float] = '#c4c4c4'
    alpha: float = 1.0

    def set_geometry(self,
                     size: float | tuple[float, float] | tuple[float, float, float],
                     position: tuple[float, float, float] | None = None,
                     orientation: tuple[float, float, float, float] | None = None,
                     center_location: str | None = None,
                     ) -> None:

        self.size = size

        if position is not None:
            self.position = position

        if orientation is not None:
            self.orientation = orientation

        if center_location is not None:
            self.center_location = center_location

    def set_dynamics(self,
                     mass: float | None,
                     center_of_mass: tuple[float, float, float] | None = None
                     ) -> None:

        if mass is not None:
            self.mass = mass

        if center_of_mass is not None:
            self.center_of_mass = center_of_mass

    def set_appearance(self,
                       color: str | tuple[float, float, float] | None,
                       alpha: float | None = None,
                       ) -> None:

        if color is not None:
            self.color = color

        if alpha is not None:
            self.alpha = alpha
