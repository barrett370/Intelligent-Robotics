import unittest

from src.foo.main import *


class MyTestCase(unittest.TestCase):
    def test_something(self):
        self.assertEqual(True, to_test())

    def test_add_correct(self):
        self.assertEqual(3, add(1, 2))

    def test_add_wrong(self):
        self.assertNotEqual(3, add(1, 1))
