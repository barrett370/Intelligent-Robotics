import unittest
from foo.main import *


class MyTestCase(unittest.TestCase):
    def test_something(self):
        self.assertEqual(False, to_test())


if __name__ == '__main__':
    unittest.main()
