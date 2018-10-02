import unittest
from sphero_sprk.sphero import DelegateObj
import struct

class MyTestCase(unittest.TestCase):

    tuple_3_result = 0
    tuple_2_result = 0
    tuple_2a_result = 0
    process_1_result = 0

    @staticmethod
    def process_3_tuple(data):

        val = struct.unpack('>hhh', data)

        MyTestCase.tuple_3_result = (val[0], val[1], val[2])

    @staticmethod
    def process_2_tuple(data):
        val = struct.unpack('>hh', data)

        MyTestCase.tuple_2_result = (val[0], val[1])

    @staticmethod
    def process_2_tuple2(data):

        val = struct.unpack('>hh', data)

        MyTestCase.tuple_2a_result = (val[0], val[1])

    @staticmethod
    def process_1(data):
        val = struct.unpack('>h', data)

        MyTestCase.process_1_result = val[0]

    def test_process_sensor_pkt(self):

        data = b'\xff\xfe\x03\x00\x07\xff0\x00[\x0f\xefm'

        masks = [
            {'len':3, 'callback':MyTestCase.process_3_tuple}
        ]
        s = DelegateObj(None, None)
        expected3 = (-208, 91, 4079)

        s.process_sensor_pkt(masks, data)

        self.assertEqual(expected3, MyTestCase.tuple_3_result)

    def test_process_sensor_pkt_long(self):

        #data = b'\xff\xfe\x03\x00\x0b\x00\x00\x00\x00\x00d\x00\x00\x00\x00\x8d'
        data = b'\xff\xfe\x03\x00\x0b\xde\xad\xbe\xef\x00\x42\xde\xad\xbe\xef\x8d'

        masks = [
            {'len': 2, 'callback':MyTestCase.process_2_tuple},
            {'len': 1, 'callback': MyTestCase.process_1},
            {'len': 2, 'callback': MyTestCase.process_2_tuple2},
        ]
        s = DelegateObj(None, None)
        expected2a = (-8531, -16657)
        expected1 = 66
        expected2b = (-8531, -16657)

        s.process_sensor_pkt(masks, data)

        self.assertEqual(expected2a, MyTestCase.tuple_2_result)
        self.assertEqual(expected1, MyTestCase.process_1_result)
        self.assertEqual(expected2b, MyTestCase.tuple_2a_result)


    def test_verify_checksum(self):
        # Arrange
        input = b'\xff\xfe\x03\x00\x07\xff0\x00[\x0f\xefm'

        s = DelegateObj(None, None)
        expected = True

        # Act
        actual = s.verify_checksum(input)

        # Assert
        self.assertEqual(expected, actual)

    def test_verify_checksum_malformed(self):
        # Arrange
        input = b'\xff\xfe\x03\x00\x07\xef0\x00[\x0f\xefm'

        s = DelegateObj(None, None)
        expected = False

        # Act
        actual = s.verify_checksum(input)

        # Assert
        self.assertEqual(expected, actual)

if __name__ == '__main__':
    unittest.main()
