import json,os,tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,IncludeLaunchDescription,SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from rc_tank_sim.model import generate

def setup(context):
    share=Path(get_package_share_directory('rc_tank_sim'));profile=LaunchConfiguration('profile').perform(context)
    config=Path(LaunchConfiguration('config').perform(context));c=json.loads(config.read_text())
    standalone=LaunchConfiguration('standalone').perform(context).lower()=='true'
    sensors=LaunchConfiguration('sensors').perform(context).lower()=='true'
    cameras=sensors and LaunchConfiguration('cameras').perform(context).lower()=='true'
    lidar=sensors and LaunchConfiguration('lidar').perform(context).lower()=='true'
    visual_detail=LaunchConfiguration('visual_detail').perform(context)
    show_rays=LaunchConfiguration('show_lidar_rays').perform(context).lower()=='true'
    headless=LaunchConfiguration('headless_rendering').perform(context).lower()=='true'
    rviz=LaunchConfiguration('rviz').perform(context).lower()=='true'
    gui=LaunchConfiguration('gui').perform(context).lower()=='true'
    out=Path(tempfile.mkdtemp(prefix='rc_tank_'))
    urdf,sdf=generate(config,profile,share/'meshes',out,sensors,visual_detail,cameras,lidar)
    actions=[];use_sim=not standalone
    controller_params={k:c[k] for k in ('command_timeout','gear_low','gear_high','sprocket_radius','motor_rpm','track_separation')}
    controller_params['use_sim_time']=use_sim
    actions.extend([
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':urdf.read_text(),'use_sim_time':use_sim,'publish_frequency':20.0}],output='screen'),
        Node(package='rc_tank_sim',executable='firmware_controller',parameters=[controller_params],output='screen'),
        Node(package='rc_tank_sim',executable='state_visualizer',parameters=[{'use_sim_time':use_sim,'standalone':standalone,'config':str(config),'profile':profile,'show_lidar_rays':show_rays}],output='screen')])
    if not standalone:
        world=(share/'worlds/test_course.sdf').read_text().replace('@STEP@','0.0001' if profile=='detailed' else '0.001')
        if not (cameras or lidar):
            tree=ET.fromstring(world);w=tree.find('world')
            for plugin in list(w.findall('plugin')):
                if plugin.attrib.get('name')=='gz::sim::systems::Sensors':w.remove(plugin)
            world=ET.tostring(tree,encoding='unicode')
        (out/'world.sdf').write_text(world)
        plugin_dir=str(Path(get_package_prefix('rc_tank_sim'))/'lib')
        actions.append(SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',plugin_dir+os.pathsep+os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')))
        gzshare=Path(get_package_share_directory('ros_gz_sim'))
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gzshare/'launch/gz_sim.launch.py')),launch_arguments={'gz_args':('-r ' if gui else ('-r -s --headless-rendering ' if headless and (cameras or lidar) else '-r -s '))+str(out/'world.sdf')}.items()))
        actions.append(Node(package='ros_gz_sim',executable='create',arguments=['-world','tank_world','-name','rc_tank','-file',str(sdf),'-z','0.105'],output='screen'))
        bridges=[]
        def bridge(ros,gz,rt,gt,direction,**kw):bridges.append(dict(ros_topic_name=ros,gz_topic_name=gz,ros_type_name=rt,gz_type_name=gt,direction=direction,**kw))
        bridge('/clock','/tank/clock','rosgraph_msgs/msg/Clock','gz.msgs.Clock','GZ_TO_ROS',subscriber_queue=1,publisher_queue=1)
        bridge('/joint_states','/tank/joint_states','sensor_msgs/msg/JointState','gz.msgs.Model','GZ_TO_ROS',subscriber_queue=1,publisher_queue=1)
        bridge('/odom','/tank/odom','nav_msgs/msg/Odometry','gz.msgs.Odometry','GZ_TO_ROS')
        bridge('/tf','/tank/odom_tf','tf2_msgs/msg/TFMessage','gz.msgs.Pose_V','GZ_TO_ROS')
        for side in ('left','right'):bridge(f'/tank/motor_{side}',f'/tank/motor_{side}','std_msgs/msg/Float64','gz.msgs.Double','ROS_TO_GZ')
        bridge('/tank/gear_internal','/tank/gear_internal','std_msgs/msg/Int32','gz.msgs.Int32','ROS_TO_GZ')
        bridge('/tank/led_colors','/tank/led_colors','std_msgs/msg/String','gz.msgs.StringMsg','ROS_TO_GZ')
        if profile=='detailed':
            bridge('/tank/shoe_poses','/tank/shoe_poses','geometry_msgs/msg/PoseArray','gz.msgs.Pose_V','GZ_TO_ROS',subscriber_queue=1,publisher_queue=1)
            bridge('/tank/closure_error','/tank/closure_error','std_msgs/msg/Float64','gz.msgs.Double','GZ_TO_ROS')
        if lidar:
            bridge('/scan','/scan','sensor_msgs/msg/LaserScan','gz.msgs.LaserScan','GZ_TO_ROS',qos_profile='SENSOR_DATA',subscriber_queue=2,publisher_queue=2)
        if cameras:
            for side in ('left','right'):
                bridge(f'/camera/{side}/image_raw',f'/camera/{side}/image_raw','sensor_msgs/msg/Image','gz.msgs.Image','GZ_TO_ROS',qos_profile='SENSOR_DATA',subscriber_queue=1,publisher_queue=1)
                bridge(f'/camera/{side}/camera_info_raw',f'/camera/{side}/camera_info','sensor_msgs/msg/CameraInfo','gz.msgs.CameraInfo','GZ_TO_ROS',qos_profile='SENSOR_DATA',subscriber_queue=1,publisher_queue=1)
        # JSON is valid YAML and avoids an additional generation dependency.
        bridgefile=out/'bridge.yaml';bridgefile.write_text(json.dumps(bridges,indent=2))
        actions.append(Node(package='ros_gz_bridge',executable='parameter_bridge',parameters=[{'config_file':str(bridgefile),'use_sim_time':True}],output='screen'))
    if rviz:actions.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'rviz/tank.rviz')],parameters=[{'use_sim_time':use_sim}],output='screen'))
    return actions

def generate_launch_description():
    share=Path(get_package_share_directory('rc_tank_sim'))
    return LaunchDescription([
        DeclareLaunchArgument('profile',default_value='light',choices=['light','detailed']),
        DeclareLaunchArgument('config',default_value=str(share/'config/robot.json')),
        DeclareLaunchArgument('standalone',default_value='false',description='RViz-only kinematic preview, no simulated sensors or contacts'),
        DeclareLaunchArgument('sensors',default_value='true'),
        DeclareLaunchArgument('cameras',default_value='false',description='Enable stereo rendering; off by default for WSL performance'),
        DeclareLaunchArgument('lidar',default_value='true'),
        DeclareLaunchArgument('visual_detail',default_value='simple',choices=['simple','stl']),
        DeclareLaunchArgument('show_lidar_rays',default_value='true'),
        DeclareLaunchArgument('headless_rendering',default_value='false',description='Use EGL when gui=false; WSLg display works without EGL'),
        DeclareLaunchArgument('rviz',default_value='true'),DeclareLaunchArgument('gui',default_value='true'),OpaqueFunction(function=setup)])
