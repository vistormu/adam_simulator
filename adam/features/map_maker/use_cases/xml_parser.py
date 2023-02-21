class XMLParser:
    @staticmethod
    def create_attribute(name: str, value: float | tuple | int | str) -> str:
        if type(value) is tuple:
            value_text: str = " ".join(map(str, value))
        else:
            value_text: str = str(value)

        return f'{name}="{value_text}"'

    @staticmethod
    def create_element(tag: str, attributes: list[str] | None, children: list[str] | None) -> str:
        element: str = ''
        if children is None and attributes is not None:
            element = f'<{tag} {" ".join(attributes)}/>'

        if children is not None and attributes is None:
            children_text: str = "\n".join(children)
            element = f'<{tag}>\n{children_text}\n</{tag}>'

        if children is not None and attributes is not None:
            children_text: str = "\n".join(children)
            element = f'<{tag} {" ".join(attributes)}>\n{children_text}\n</{tag}>'

        return element
