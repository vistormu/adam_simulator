from dataclasses import dataclass


@dataclass
class Body:
    '''
    the class Body is a dataclass containing all the information that a MuJoCo body can adopt

    Attributes
    ----------
    name : str
        the unique name identifier of the body

    type : str
        the type of the body (cylinder, box, sphere....)

    position : tuple[float, float, float]
        the position of the body relative to the origin

    orientation : tuple[float, float, float, float]
        the orientation of the body in quaternions

    size : float | tuple[float, float] | tuple[float, float, float]
         the size of the body. The representation depends on the body type

    mass : float
        the mass of the body expressed in kg

    center_of_mass : tuple[float, float, float]
        the location of the center of mass referred to the geometric center of the body

    color : str | tuple[float, float, float]
        the color of the body expressed in hexadecimal or rgb

    alpha : float
        the transparency of the body

    Methods
    -------
    set_geometry:
        sets all the geometry parameters of the body

    set_dynamics:
        sets all the dynamic parameters of the body

    set_appearance:
        sets all the appearance parameters of the body

    Notes
    -----
    At least the size of the object must be specified
    '''
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
        '''
        sets all the geometry parameters of the body

        Parameters
        ----------
        size : float | tuple[float, float] | tuple[float, float, float]
            the size of the body. It depends on the object type

        position : tuple[float, float, float], optional
            the position of the body relative to the origin. (0.0, 0.0, 0.0) by default

        orientation : tuple[float, float, float, float], optional
            the orientation of the body in quaternions. (1.0, 0.0, 0.0, 0.0) by default

        center_location: str, optional
            a WIP attribute
        '''
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
        '''
        sets all the dynamic parameters of the body

        Parameters
        ----------
        mass : float, optional
            the mass of the body expressed in kg. 1.0 by default

        center_of_mass : tuple[float, float, float], optional
            the location of the center of mass referred to the geometric center of the body. (0.0, 0.0, 0.0) by default
        '''
        if mass is not None:
            self.mass = mass

        if center_of_mass is not None:
            self.center_of_mass = center_of_mass

    def set_appearance(self,
                       color: str | tuple[float, float, float] | None,
                       alpha: float | None = None,
                       ) -> None:
        '''
        sets all the appearance parameters of the body

        Parameters
        ----------
        color : str | tuple[float, float, float], optional
            the color of the body expressed in hexadecimal or rgb.'#c4c4c4' by default

        alpha : float, optional
            the transparency of the body. 1.0 by default
        '''
        if color is not None:
            self.color = color

        if alpha is not None:
            self.alpha = alpha
