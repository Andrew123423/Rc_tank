#!/usr/bin/env python3
"""Launch wiring checks with mocked launch actions; no claim of ROS execution."""
import importlib.util,json,sys,tempfile,types,unittest
from pathlib import Path
import xml.etree.ElementTree as E
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
def action(kind):
    def create(*args,**kwargs):return {'kind':kind,'args':args,**kwargs}
    return create
for name in ('ament_index_python','ament_index_python.packages','launch','launch.actions','launch.launch_description_sources','launch.substitutions','launch_ros','launch_ros.actions'):
    sys.modules[name]=types.ModuleType(name)
p=sys.modules['ament_index_python.packages'];p.get_package_share_directory=lambda name:str(ROOT if name=='rc_tank_sim' else ROOT/'fake_gz');p.get_package_prefix=lambda name:str(ROOT.parent)
sys.modules['launch'].LaunchDescription=lambda x:x
for name in ('DeclareLaunchArgument','OpaqueFunction','IncludeLaunchDescription','SetEnvironmentVariable'):setattr(sys.modules['launch.actions'],name,action(name))
sys.modules['launch.launch_description_sources'].PythonLaunchDescriptionSource=lambda x:x
class Config:
    def __init__(self,key):self.key=key
    def perform(self,context):return context[self.key]
sys.modules['launch.substitutions'].LaunchConfiguration=Config
sys.modules['launch_ros.actions'].Node=action('Node')
spec=importlib.util.spec_from_file_location('tank_launch',ROOT/'launch/sim.launch.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
class LaunchTests(unittest.TestCase):
    def setup_actions(self,**overrides):
        context=dict(profile='light',config=str(ROOT/'config/robot.json'),standalone='false',sensors='true',cameras='false',lidar='true',rviz='true',gui='true',visual_detail='simple',show_lidar_rays='true',headless_rendering='false');context.update(overrides)
        actions=module.setup(context)
        bridge=next((x for x in actions if x.get('package')=='ros_gz_bridge'),None)
        if bridge:
            config=Path(bridge['parameters'][0]['config_file']);self.addCleanup(lambda:__import__('shutil').rmtree(config.parent,ignore_errors=True));data=json.loads(config.read_text())
        else:data=[]
        return actions,data
    def test_default_lidar_only_and_clock(self):
        actions,data=self.setup_actions();topics={b['ros_topic_name'] for b in data};self.assertIn('/scan',topics);self.assertNotIn('/camera/left/image_raw',topics)
        self.assertEqual(next(b for b in data if b['ros_topic_name']=='/clock')['gz_topic_name'],'/tank/clock')
        rsp=next(x for x in actions if x.get('package')=='robot_state_publisher');self.assertEqual(rsp['parameters'][0]['publish_frequency'],20.)
    def test_camera_switch(self):
        _,data=self.setup_actions(cameras='true');self.assertIn('/camera/right/image_raw',{b['ros_topic_name'] for b in data})
    def test_master_sensor_off(self):
        _,data=self.setup_actions(sensors='false',cameras='true');topics={b['ros_topic_name'] for b in data};self.assertNotIn('/scan',topics);self.assertFalse(any('camera' in x for x in topics))
    def test_detailed_diagnostics(self):
        _,data=self.setup_actions(profile='detailed');topics={b['ros_topic_name'] for b in data};self.assertIn('/tank/shoe_poses',topics);self.assertIn('/tank/closure_error',topics)
    def test_standalone(self):
        actions,data=self.setup_actions(standalone='true');self.assertFalse(data);self.assertFalse(any(x.get('package')=='ros_gz_sim' for x in actions))
if __name__=='__main__':unittest.main(verbosity=2)
