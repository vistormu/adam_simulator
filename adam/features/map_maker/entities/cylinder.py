from .body import Body


class Cylinder(Body):
    def __init__(self, name: str) -> None:
        super().__init__()
        self.name = name
        self.type = 'cylinder'

    def set_geometry(self, size: tuple[float, float], position: tuple[float, float, float] | None = None, orientation: tuple[float, float, float, float] | None = None, center_location: str | None = None) -> None:
        if type(size) is not tuple and len(size) != 2:
            raise TypeError(f'the size of a {__class__.__name__} instance must be a tuple[float, float]. Received {type(size)}')

        radius: float = size[0]
        cylinder_half_height: float = size[1]/2.0

        real_size: tuple[float, float] = (radius, cylinder_half_height)

        if position is not None:
            position = (position[0], position[1], position[2]+cylinder_half_height)

        return super().set_geometry(real_size, position, orientation, center_location)
