#!/usr/bin/env python3
import copy,json,math,struct,sys,tempfile,unittest
from pathlib import Path
import xml.etree.ElementTree as E
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from rc_tank_sim.model import generate,belt_path,validate

class ModelTests(unittest.TestCase):
    def setUp(self):
        self.c=json.loads((ROOT/'config/robot.json').read_text());self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
    def roots(self,profile,sensors=True):
        u,s=generate(ROOT/'config/robot.json',profile,ROOT/'meshes',self.tmp.name,sensors);return E.parse(u).getroot(),E.parse(s).getroot().find('model')
    def test_urdf_is_connected_tree(self):
        for mode in ('light','detailed'):
            u,_=self.roots(mode);links={n.attrib['name'] for n in u.findall('link')};children={}
            for j in u.findall('joint'):
                p=j.find('parent').attrib['link'];c=j.find('child').attrib['link'];self.assertIn(p,links);self.assertIn(c,links);self.assertNotIn(c,children);children[c]=p
            self.assertEqual(links-set(children),{'base_link'})
            for c in children:
                seen=set()
                while c in children:self.assertNotIn(c,seen);seen.add(c);c=children[c]
    def test_profiles_physical_links(self):
        _,l=self.roots('light');u,h=self.roots('detailed')
        self.assertEqual(len(h.findall('link'))-len(l.findall('link')),214)
        self.assertEqual(len([x for x in h.findall('link') if '_shoe_' in x.attrib['name']]),214)
        self.assertFalse(any('_shoe_' in x.attrib['name'] for x in u.findall('link')))
        self.assertEqual(len([j for j in h.findall('joint') if j.attrib['type']=='prismatic']),8)
        self.assertEqual(h.findtext('self_collide'),'true')
    def test_native_open_chains_and_single_closure(self):
        _,s=self.roots('detailed')
        for side in ('left','right'):
            joints=[j for j in s.findall('joint') if j.attrib['name'].startswith(side+'_pin_')]
            self.assertEqual(len(joints),106)
            children={j.findtext('child') for j in joints}
            self.assertNotIn(side+'_shoe_0',children)
            self.assertIn(side+'_shoe_106',children)
            self.assertTrue(all(j.attrib['type']=='revolute' for j in joints))
            self.assertTrue(all(j.find('pose').attrib['relative_to']=='__model__' for j in joints))
    def test_joint_state_rate_and_proxy(self):
        u,s=self.roots('light')
        plugin=s.find("plugin[@name='gz::sim::systems::JointStatePublisher']")
        self.assertEqual(float(plugin.findtext('update_rate')),20)
        names={j.attrib['name'] for j in u.findall('joint') if j.attrib['type']!='fixed'}
        self.assertEqual({e.text for e in plugin.findall('joint_name')},names)
        meshes=u.findall("link[@name='base_link']/visual/geometry/mesh")
        self.assertEqual(len(meshes),1);self.assertTrue(meshes[0].attrib['filename'].endswith('body_proxy.stl'))
    def test_sensor_independent_switches(self):
        u,s=generate(ROOT/'config/robot.json','light',ROOT/'meshes',self.tmp.name,cameras=False,lidar=True)
        sensors=E.parse(s).findall('.//sensor');self.assertEqual(len(sensors),1);self.assertEqual(sensors[0].attrib['type'],'gpu_lidar')
        self.assertEqual(sensors[0].findtext('visualize'),'true')
        u,s=generate(ROOT/'config/robot.json','light',ROOT/'meshes',self.tmp.name,cameras=True,lidar=False,visual_detail='stl')
        self.assertEqual(len(E.parse(s).findall('.//sensor')),2)
        self.assertEqual(len(E.parse(u).findall("link[@name='base_link']/visual/geometry/mesh")),4)
    def test_collision_primitives_only(self):
        for mode in ('light','detailed'):
            _,s=self.roots(mode);self.assertFalse(s.findall('.//collision/geometry/mesh'))
            for dynamics in s.findall('.//joint/axis/dynamics'):
                tags=[child.tag for child in dynamics];self.assertEqual(len(tags),len(set(tags)))
            for link in s.findall('link'):
                self.assertGreater(float(link.findtext('inertial/mass')),0)
                for axis in ('ixx','iyy','izz'):self.assertGreater(float(link.findtext('inertial/inertia/'+axis)),0)
    def test_sensors_and_optical_frames(self):
        u,s=self.roots('light');sensors=s.findall('.//sensor');self.assertEqual(len(sensors),3)
        self.assertEqual({x.attrib['type'] for x in sensors},{'camera','gpu_lidar'})
        joints={j.attrib['name']:j for j in u.findall('joint')}
        for side in ('left','right'):
            a=[float(v) for v in joints[f'camera_{side}_optical_frame_joint'].find('origin').attrib['rpy'].split()]
            self.assertAlmostEqual(a[0],-math.pi/2,places=7);self.assertAlmostEqual(a[2],-math.pi/2,places=7)
        l=s.find("link[@name='camera_left_frame']/pose").text.split();r=s.find("link[@name='camera_right_frame']/pose").text.split()
        self.assertAlmostEqual(float(l[1])-float(r[1]),self.c['stereo_baseline'])
    def test_sensor_toggle(self):
        _,s=self.roots('light',False);self.assertFalse(s.findall('.//sensor'))
    def test_belt_pitch_closure_and_envelope(self):
        path=belt_path(self.c);self.assertEqual(len(path),107)
        distances=[math.hypot(x-path[(i+1)%107][0],z-path[(i+1)%107][1]) for i,(x,z,_) in enumerate(path)]
        self.assertGreater(min(distances),.0108);self.assertLess(max(distances),.0111)
        extent=2*(self.c['half_axle_spacing']+self.c['sprocket_radius']);self.assertAlmostEqual(extent,.54774)
        self.assertAlmostEqual(self.c['track_separation']+self.c['track_width'],.397)
    def test_source_assets_reduced(self):
        manifest=json.loads((ROOT/'docs/mesh_provenance.json').read_text())
        self.assertLess(sum(x['output_triangles'] for x in manifest),9000)
        for item in manifest:
            data=(ROOT/'meshes'/item['output']).read_bytes();n=struct.unpack_from('<I',data,80)[0];self.assertEqual(len(data),84+50*n);self.assertEqual(n,item['output_triangles'])
    def test_bad_config_rejected(self):
        for k,v in [('gear_low',1),('links_per_side',2.5),('body_mass',0),('hinge_stiffness',-1),('camera_hfov',4),('camera_rate',float('nan'))]:
            c=copy.deepcopy(self.c);c[k]=v
            with self.assertRaises(ValueError):validate(c)
    def test_actuator_joint_names(self):
        _,s=self.roots('detailed');joints={j.attrib['name'] for j in s.findall('joint')}
        for side in ('left','right'):
            for suffix in ('sprocket','tensioner','suspension_0'):self.assertIn(side+'_'+suffix+'_joint',joints)
        self.assertIn('shift_servo_joint',joints)
    def test_led_count_and_gear_ratios(self):
        _,s=self.roots('light');base=s.find("link[@name='base_link']")
        self.assertEqual(len([v for v in base.findall('visual') if v.attrib['name'].startswith('led_')]),24)
        self.assertAlmostEqual(self.c['gear_high'],48/25)

if __name__=='__main__':unittest.main(verbosity=2)
