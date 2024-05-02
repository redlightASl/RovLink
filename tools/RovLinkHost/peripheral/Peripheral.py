from .joystick import Joystick
from .keyboard import Keyboard


class Peripheral(object):
    def __init__(self, periph_type="Joystick", **kwargs) -> None:
        self._periph_type = periph_type
        if self._periph_type == "Joystick":
            self._dev = Joystick(
                max_val=kwargs["max_val"], debug=kwargs["debug"])
            self._cmd = self._dev.get_value()
        elif self._periph_type == "Keyboard":
            self._dev = Keyboard()  # TODO
        else:
            print("Error: Peripheral type not supported!")

    def get_cmd(self):
        if self._periph_type == "Joystick":
            return next(self._cmd)


if __name__ == "__main__":
    periph = Peripheral("Joystick", max_val=14, debug=False)
    while True:
        cmd = periph.get_cmd()
        print(cmd)
