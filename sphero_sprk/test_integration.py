import unittest
from sphero import Sphero
import time
import struct

class MyTestCase(unittest.TestCase):

    SPHERO = "D2:E6:0C:03:02:7D"
    #SPHERO = "F5:EC:FB:94:BD:CE"

    current_position = None

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


    def print_xy(self, data):
        if(len(data) == 0):
            return

        val = struct.unpack('>hh', data)

        x = val[0] # assuming filtered values
        y = val[1]

        print("(" + str(x) + "," + str(y) + ")")
        MyTestCase.current_position = (x,y)

    def test_locator(self):
        orb = Sphero(MyTestCase.SPHERO)
        orb.connect()

        orb.set_stream_callback('odometer', callback=self.print_xy, mask_id=2)
        orb.update_streaming(rate=10)

        time.sleep(2)

        before = MyTestCase.current_position

        orb.config_locator(10,50,0)
        orb.ping()

        time.sleep(2)

        orb.config_locator(50, -10, 0)
        orb.ping()

        middle = MyTestCase.current_position

        orb.ping()
        time.sleep(2)
        orb.ping()
        time.sleep(2)

        after = MyTestCase.current_position

        self.assertNotEquals(before, middle)
        self.assertNotEquals(middle, after)

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
