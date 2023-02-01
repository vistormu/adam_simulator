from .formatter import Formatter


class Logger:
    @staticmethod
    def info(*message, sep: str = '') -> None:
        prefix: str = Formatter.green(Formatter.bold('[INFO]'))
        output_message: str = Formatter.green(sep.join(map(str, message)))
        print(f'{prefix} {output_message}')

    @staticmethod
    def debug(*message, sep: str = '') -> None:
        prefix: str = Formatter.blue(Formatter.bold('[DEBUG]'))
        output_message: str = Formatter.blue(sep.join(map(str, message)))
        print(f'{prefix} {output_message}')

    @staticmethod
    def warning(*message, sep: str = '') -> None:
        prefix: str = Formatter.yellow(Formatter.bold('[WARNING]'))
        output_message: str = Formatter.yellow(sep.join(map(str, message)))
        print(f'{prefix} {output_message}')

    @staticmethod
    def error(*message, sep: str = '') -> None:
        prefix: str = Formatter.red(Formatter.bold('[ERROR]'))
        output_message: str = Formatter.red(sep.join(map(str, message)))
        print(f'{prefix} {output_message}')