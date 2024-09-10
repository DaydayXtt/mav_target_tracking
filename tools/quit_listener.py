from pynput import keyboard
import numpy as np
class QuitListener():
    def __init__(self):
        self.move_dir = np.array([0., 0.])
        def on_press(key):
            if key == keyboard.Key.esc:
                return False
            else:
                print("Press 'Esc' to exit...")
                if key.char == 'w':
                    # print("Up arrow key pressed")
                    self.move_dir[1] = -0.2
                elif key.char == 's':
                    # print("Down arrow key pressed")
                    self.move_dir[1] = 0.2
                elif key.char == 'a':
                    # print("Left arrow key pressed")
                    self.move_dir[0] = -0.2
                elif key.char == 'd':
                    # print("Right arrow key pressed")
                    self.move_dir[0] = 0.2
        self._listener = keyboard.Listener(on_press=on_press)
        self._listener.start() 

    def check_quit(self):
        if self._listener.running:
            return False
        else:
            return True
        
    def stop_listener(self):
        self._listener.stop()

    # def on_activate_h():
    #     print('<ctrl>+<alt>+h pressed')

    # def on_activate_i():
    #     print('<ctrl>+<alt>+i pressed')

    # with keyboard.GlobalHotKeys({
    #         '<ctrl>+<alt>+h': on_activate_h,
    #         '<ctrl>+<alt>+i': on_activate_i}) as h:
    #     h.join()
    