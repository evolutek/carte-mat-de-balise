import adafruit_vl53l0x
from adafruit_mcp230xx.mcp23017 import MCP23017
import digitalio
import board
import busio
import time
import neopixel

MAX_DIST = 1000

class MDP:

    def __init__(self, nbr_capteur):
        self.coef = 255 / (MAX_DIST / 2)
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.nb_capteur = nbr_capteur
        self.sensors = []
        self.mcp = MCP23017(self.i2c, address=0x27)
        self.init_sensor()
        
        
    def init_sensor(self):

        for i in range(0, self.nb_capteur + 1):
            self.mcp.get_pin(i + 8).switch_to_output()

        for i in range(0, self.nb_capteur):
            print("INIT_Sensor: {0}".format(i + 1))
            self.mcp.get_pin(i + 8).value = True
            self.sensors.append(adafruit_vl53l0x.VL53L0X(self.i2c, address=0x29))
            self.sensors[i].set_address(0x29 + i + 1)
        
        self.pixel = neopixel.NeoPixel(board.D18, self.nb_capteur, brightness=10)


    def run(self):
        while True:
            for i in range(0, self.nb_capteur):
                self.dist = self.sensors[i].range
                if self.dist > MAX_DIST:
                    self.pixel[i] = (0,0,0)
                else:
                    r = 255 - self.dist * self.coef
                    g = 255 - abs(MAX_DIST / 2 - self.dist) * self.coef
                    b = 255 - (MAX_DIST - self.dist) * self.coef
                    self.pixel[i] = (int(r if r >= 0 else 0),\
                        int(g if g >= 0 else 0),\
                        int(b if b >= 0 else 0))
                print("Range: {0}".format(self.sensors[i].range))

mdp = MDP(2)
mdp.run()
