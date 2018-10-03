import unittest
from sphero import Sphero
import time

class MyTestCase(unittest.TestCase):

    SPHERO = "D2:E6:0C:03:02:7D"
    #SPHERO = "F5:EC:FB:94:BD:CE"

    def test_connect(self):
        orb = Sphero(MyTestCase.SPHERO)
        result = orb.connect()
        self.assertTrue(result)

    def test_set_color(self):
        orb = Sphero(MyTestCase.SPHERO)
        orb.connect()

        orb.set_rgb_led(255,0,0)

    def test_roll(self):
        orb = Sphero(MyTestCase.SPHERO)
        orb.connect()

        orb.roll(10,0)

    def test_set_heading(self):
        orb = Sphero(MyTestCase.SPHERO)
        orb.connect()

        orb.set_tail_light(255)

        orb.set_stabilization(False)

        print("Move Sphero so tail is heading to -y")
        input("Press Enter to continue...")

        orb.set_stabilization(True)

        orb.set_tail_light(0)
        time.sleep(3)

    def test_set_heading(self):
        orb = Sphero(MyTestCase.SPHERO)
        orb.connect()

        orb.set_tail_light(255)

        orb.set_heading(0)
        time.sleep(2)

        orb.set_heading(90)
        time.sleep(2)

        orb.set_heading(180)
        time.sleep(2)

        orb.set_heading(270)
        time.sleep(2)

        orb.set_tail_light(0)
        time.sleep(3)

    def test_set_heading2(self):
        orb = Sphero(MyTestCase.SPHERO)
        orb.connect()

        for angle in range(0,360,90):

            orb.roll(30,90)
            time.sleep(3)
            orb.set_heading(angle)
            time.sleep(1)
            orb.roll(30, 90)
            time.sleep(3)

if __name__ == '__main__':
    unittest.main()
