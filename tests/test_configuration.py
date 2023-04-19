import unittest
from adam_sim.entities import Configuration


class TestConfiguration(unittest.TestCase):
    def test_add(self):
        self.assertTupleEqual(Configuration(0, 1, 2, 3, 4, 5)+Configuration(1, 2, 3, 4, 5, 6), Configuration(1, 3, 5, 7, 9, 11))
        self.assertTupleEqual((Configuration(0.32, 1.45, 2.98, 3.67, 4.09, 5.01)+Configuration(0.0, 0.0, 0.0, 0.0, 0.0, 0.1)).round(), Configuration(0.32, 1.45, 2.98, 3.67, 4.09, 5.11))


if __name__ == '__main__':
    unittest.main()
