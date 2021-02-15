import board
import time
from Mouse import Mouse

bob = Mouse([board.D12, board.D11, board.D10])
bob.moveForward(0.25)
time.sleep(1)
bob.stop()
time.sleep(1)
bob.turn("right")
time.sleep(1)
bob.turn("right")
time.sleep(1)
bob.turn("left")
time.sleep(1)
bob.turn("right")
time.sleep(1)
bob.moveForward(0.25)
time.sleep(1)
bob.stop()
