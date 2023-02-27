from .body import Body


class Box(Body):
    '''
    the Box class inherits from the ~.entities.Body class

    Parameters
    ----------
    name : str
        a unique string identifier
    '''

    def __init__(self, name: str) -> None:
        super().__init__()
        self.name = name
        self.type = 'box'

    def set_geometry(self, size: tuple[float, float, float], position: tuple[float, float, float] | None = None, orientation: tuple[float, float, float, float] | None = None, center_location: str | None = None) -> None:
        if type(size) is not tuple and len(size) != 3:
            raise TypeError(f'the size of a {__class__.__name__} instance must be a tuple[float, float, float]. Received {type(size)}')

        real_size: tuple[float, float, float] = (size[0]/2.0, size[1]/2.0, size[2]/2.0)

        if position is not None:
            position = (position[0], position[1], position[2]+size[2]/2.0)

        return super().set_geometry(real_size, position, orientation, center_location)
