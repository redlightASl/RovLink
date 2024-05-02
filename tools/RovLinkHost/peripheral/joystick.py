import pygame


class Joystick(object):
    def __init__(self, max_val=14, debug=True) -> None:
        self._recv_done = False  # !Not Used
        if max_val > 20:
            print("max_val must be less than 20")
            max_val = 20

        self._max_val = max_val
        self._debug = debug
        self._value_list = [0]*20
        self._clock = pygame.time.Clock()
        pygame.init()
        pygame.joystick.init()

    def get_value(self):
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.recv_done = True

            joystick_count = pygame.joystick.get_count()

            # For each joystick:
            for i in range(joystick_count):
                joystick = pygame.joystick.Joystick(i)
                joystick.init()

                axes = joystick.get_numaxes()
                for i in range(axes):
                    axis = joystick.get_axis(i)
                    self._value_list[i] = round(axis, 4)

                buttons = joystick.get_numbuttons()
                for i in range(buttons):
                    button = joystick.get_button(i)
                    self._value_list[i+4] = button

                if self._debug:
                    print(self._value_list[0:self._max_val])
                yield self._value_list[0:self._max_val]


if __name__ == '__main__':
    joystick = Joystick(max_val=8, debug=True)
    value = joystick.get_value()
    while True:
        temp_value = next(value)
