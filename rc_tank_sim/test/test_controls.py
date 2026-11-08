#!/usr/bin/env python3
import math,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from rc_tank_sim.teleop_state import KeyState
from rc_tank_sim.visual_geometry import shoe_triangles,scan_segments

class Controls(unittest.TestCase):
    def test_simultaneous_forward_right(self):
        s=KeyState(.4);s.press('w');s.press('d');self.assertEqual(s.command(),(.4,-.2));s.release('d');self.assertEqual(s.command(),(.4,0));s.release('w');self.assertEqual(s.command(),(0,0))
    def test_forward_left_and_reverse_right(self):
        s=KeyState(.4);s.press('w');s.press('a');self.assertEqual(s.command(),(.4,.2));s.clear();s.press('s');s.press('d');self.assertEqual(s.command(),(-.4,-.2))
    def test_opposing_and_arrows(self):
        s=KeyState(.4)
        for k in ('w','s','a','d'):s.press(k)
        self.assertEqual(s.command(),(0,0));s.clear();s.press('Up');s.press('Right');self.assertEqual(s.command(),(.4,-.2))
    def test_repeat_does_not_toggle(self):
        s=KeyState();s.press('f');s.press('f');s.press('f');self.assertEqual(s.light,1);s.release('f');s.press('f');self.assertEqual(s.light,2)
    def test_focus_loss_and_stop(self):
        s=KeyState();s.press('w');s.press('a');s.clear();self.assertEqual(s.command(),(0,0));s.press('d');s.press('space');self.assertEqual(s.command(),(0,0));s.press('d');self.assertEqual(s.command(),(0,0));s.release('d');s.press('d');self.assertLess(s.command()[1],0)
    def test_case_and_gear(self):
        s=KeyState();s.press('W');s.press('2');self.assertEqual(s.gear,2);self.assertGreater(s.command()[0],0);s.release('W');self.assertEqual(s.command(),(0,0))
    def test_batched_shoe_transform(self):
        # 12 triangles per shoe, all are transformed with the actual quaternion.
        points=shoe_triangles((1,2,3),(0,math.sin(math.pi/4),0,math.cos(math.pi/4)),.05)
        self.assertEqual(len(points),36)
        self.assertAlmostEqual(max(p[0] for p in points)-min(p[0] for p in points),.004)
        self.assertAlmostEqual(max(p[2] for p in points)-min(p[2] for p in points),.0095)
        self.assertEqual(shoe_triangles((float('nan'),0,0),(0,0,0,1),.05),[])
    def test_scan_finite_hits_only(self):
        out=scan_segments([1,float('inf'),float('nan'),0,20,2],0,math.pi/2,.1,12)
        self.assertEqual(len(out),2);self.assertEqual(out[0][1],(1,0,0));self.assertAlmostEqual(out[1][1][1],2)
if __name__=='__main__':unittest.main(verbosity=2)
