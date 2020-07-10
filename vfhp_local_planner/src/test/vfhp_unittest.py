import unittest
from .context import Planners

import math

class TestAngleMethods(unittest.TestCase):

    def test_inRange(self):
        planner = Planners.VFHP.VFHPModel()

        self.assertTrue( planner._isInRange(0, math.radians(45), math.radians(25)) )
        self.assertTrue( planner._isInRange(0, math.radians(30), math.radians(10)) )
        self.assertTrue( planner._isInRange(0, math.radians(30), math.radians(0)) )
        self.assertTrue( planner._isInRange(0, math.radians(30), math.radians(29.9)) )

        self.assertFalse( planner._isInRange(0, math.radians(30), math.radians(35)) )
        self.assertFalse( planner._isInRange(0, math.radians(30), math.radians(-5)) )
        self.assertFalse( planner._isInRange(0, math.radians(30), math.radians(-170)) )
        self.assertFalse( planner._isInRange(0, math.radians(30), math.radians(170)) )
        self.assertFalse( planner._isInRange(0, math.radians(30), math.radians(95)) )

        self.assertTrue( planner._isInRange(math.radians(30), 0.0, math.radians(35)) )
        self.assertTrue( planner._isInRange(math.radians(30), 0.0, math.radians(-5)) )
        self.assertTrue( planner._isInRange(math.radians(30), 0.0, math.radians(-170)) )
        self.assertTrue( planner._isInRange(math.radians(30), 0.0, math.radians(170)) )
        self.assertTrue( planner._isInRange(math.radians(30), 0.0, math.radians(95)) )

        self.assertFalse( planner._isInRange(math.radians(45), 0.0, math.radians(25)) )
        self.assertFalse( planner._isInRange(math.radians(30), 0.0, math.radians(10)) )
        self.assertFalse( planner._isInRange(math.radians(30), 0.0, math.radians(0)) )
        self.assertFalse( planner._isInRange(math.radians(30), 0.0, math.radians(29.9)) )


        self.assertTrue( planner._isInRange(math.radians(170), math.radians(-170), math.radians(175)) )
        self.assertTrue( planner._isInRange(math.radians(170), math.radians(-170), math.radians(180)) )
        self.assertTrue( planner._isInRange(math.radians(170), math.radians(-170), math.radians(-180)) )
        self.assertTrue( planner._isInRange(math.radians(170), math.radians(-170), math.radians(-175)) )

        self.assertFalse( planner._isInRange(math.radians(170), math.radians(-170), math.radians(155)) )
        self.assertFalse( planner._isInRange(math.radians(170), math.radians(-170), math.radians(-155)) )
        self.assertFalse( planner._isInRange(math.radians(170), math.radians(-170), math.radians(15)) )
        self.assertFalse( planner._isInRange(math.radians(170), math.radians(-170), math.radians(-15)) )

        self.assertFalse( planner._isInRange(math.radians(-170), math.radians(170), math.radians(175)) )
        self.assertFalse( planner._isInRange(math.radians(-170), math.radians(170), math.radians(180)) )
        self.assertFalse( planner._isInRange(math.radians(-170), math.radians(170), math.radians(-180)) )
        self.assertFalse( planner._isInRange(math.radians(-170), math.radians(170), math.radians(-175)) )

        self.assertTrue( planner._isInRange(math.radians(-170), math.radians(170), math.radians(155)) )
        self.assertTrue( planner._isInRange(math.radians(-170), math.radians(170), math.radians(-155)) )
        self.assertTrue( planner._isInRange(math.radians(-170), math.radians(170), math.radians(15)) )
        self.assertTrue( planner._isInRange(math.radians(-170), math.radians(170), math.radians(-15)) )

if __name__ == '__main__':
    unittest.main()
