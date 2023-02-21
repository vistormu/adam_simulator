from .use_cases import XMLParser
from .entities import Body


class MapMaker:
    def __init__(self, filename: str) -> None:
        self.filename: str = filename
        self.bodies: list[str] = []
        self.body_names: list[str] = []

    def create_xml(self) -> None:
        content: str = XMLParser.create_element('worldbody', None, self.bodies)
        name: str = self.filename.split('/')[-1].split('.')[0]
        model_attr: str = XMLParser.create_attribute('model', name)
        content: str = XMLParser.create_element('mujoco', [model_attr], [content])

        with open(self.filename, 'w') as file:
            file.write(content)

    def add_to_scene(self, filename: str) -> None:
        raise NotImplementedError()

    def add_bodies(self, bodies: list[Body]) -> None:
        for body in bodies:
            if not body.size:
                raise ValueError(f'the body with id {body.name} must have a size')

            if body.name in self.body_names:
                raise ValueError(f'the bodies must have unique names. Name "{body.name}" is repeated')

            self.body_names.append(body.name)
            self._add_body(body)

    def _add_body(self, body: Body) -> None:
        # Geom
        type_attr: str = XMLParser.create_attribute('type', body.type)
        size_attr: str = XMLParser.create_attribute('size', body.size)
        color_attr: str = XMLParser.create_attribute('rgba', (*body.color, body.alpha) if type(body.color) is tuple else self._hex_to_rgba(body.color, body.alpha))  # type: ignore

        geom_element: str = XMLParser.create_element('geom', [type_attr, size_attr, color_attr], None)

        # Inertial
        inertial_element: str | None = None
        if body.mass:
            inertia_pos_attr: str = XMLParser.create_attribute('pos', body.center_of_mass)
            mass_attr: str = XMLParser.create_attribute('mass', body.mass)

            inertial_element = XMLParser.create_element('inertial', [mass_attr, inertia_pos_attr], None)

        # Body
        name_attr: str = XMLParser.create_attribute('name', body.name)
        position_attr: str = XMLParser.create_attribute('pos', body.position)

        elements_list = [geom_element, inertial_element]

        body_element: str = XMLParser.create_element('body', [name_attr, position_attr], [element for element in elements_list if element is not None])

        self.bodies.append(body_element)

    @staticmethod
    def _hex_to_rgba(hex_code: str, alpha: float) -> tuple[float, float, float, float]:
        hex: str = hex_code.lstrip('#')
        rgb: tuple[int, int, int] = tuple(int(hex[i:i+2], 16) for i in (0, 2, 4))

        rgba: tuple[float, float, float, float] = (round(rgb[0]/255.0, 4), round(rgb[1]/255.0, 4), round(rgb[2]/255.0, 4), alpha)

        return rgba
