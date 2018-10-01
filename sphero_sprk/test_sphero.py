import unittest
from sphero_sprk.sphero import DelegateObj

class MyTestCase(unittest.TestCase):

    def test_process_sensor_package(self):
        # Arrange
        input = b'\xff\xfe\x03\x00\x07\xff0\x00[\x0f\xefm'
        s = DelegateObj()
        expected = []

        # Act
        result = s.process_sensor_package(input)

        # Assert
        self.assertEqual(True, False)


if __name__ == '__main__':
    unittest.main()
