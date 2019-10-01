import unittest

from src.foo.main import to_test


class MyTestCase(unittest.TestCase):
    def test_something(self):
        self.assertEqual(False, to_test())


if __name__ == '__main__':
    unittest.main()
