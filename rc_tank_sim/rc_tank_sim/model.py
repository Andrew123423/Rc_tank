"""Generate URDF (tree) and native SDF (including compliant free track links).
Only Python standard library is needed. Lengths are metres, angles radians.
"""
import argparse,json,math
from pathlib import Path
import xml.etree.ElementTree as E
PI=math.pi

def el(parent,tag,text=None,**attrs):
    n=E.SubElement(parent,tag,{k:str(v) for k,v in attrs.items()})
    if text is not None:n.text=str(text)
    return n

def nums(v):return ' '.join(f'{x:.9g}' for x in v)

def belt_path(c):
    """107 equal-arc-length shoe centres and tangents, clockwise in x/z."""
    a=c['half_axle_spacing'];r=c['sprocket_radius']; L=4*a+2*PI*r
    result=[]
    for i in range(c['links_per_side']):
        s=i*L/c['links_per_side']
        if s<2*a:x,z,t=-a+s,-.05+r,0.
        elif s<2*a+PI*r:
            q=(s-2*a)/r;x,z,t=a+r*math.sin(q),-.05+r*math.cos(q),q
        elif s<4*a+PI*r:
            q=s-2*a-PI*r;x,z,t=a-q,-.05-r,PI
        else:
            q=(s-4*a-PI*r)/r;x,z,t=-a-r*math.sin(q),-.05-r*math.cos(q),PI+q
        result.append((x,z,t))
    return result

class Builder:
    def __init__(self,c,profile,mesh_dir,sensors,visual_detail="simple",cameras=True,lidar=True):
        self.c=c;self.profile=profile;self.mesh_dir=Path(mesh_dir).resolve();self.sensors=sensors;self.visual_detail=visual_detail;self.cameras=cameras;self.lidar=lidar
        self.u=E.Element('robot',name='rc_tank');self.s=E.Element('sdf',version='1.9')
        self.m=el(self.s,'model',name='rc_tank');el(self.m,'self_collide',str(profile=='detailed').lower())
        self.poses={};self.links={}
    def link(self,name,pos,mass,size=(.02,.02,.02),parent=None,jtype='fixed',axis=(0,1,0),limits=None,spring=None,urdf=True,rpy=(0,0,0)):
        self.poses[name]=pos
        s=el(self.m,'link',name=name);el(s,'pose',nums((*pos,*rpy)))
        inertia=[mass*(size[1]**2+size[2]**2)/12,mass*(size[0]**2+size[2]**2)/12,mass*(size[0]**2+size[1]**2)/12]
        ins=el(s,'inertial');el(ins,'mass',mass);ii=el(ins,'inertia')
        for k,v in zip(('ixx','iyy','izz','ixy','ixz','iyz'),(*inertia,0,0,0)):el(ii,k,max(v,1e-9) if k in ('ixx','iyy','izz') else v)
        u=el(self.u,'link',name=name) if urdf else None
        if u is not None:
            insu=el(u,'inertial');el(insu,'mass',value=mass);el(insu,'inertia',**dict(zip(('ixx','iyy','izz','ixy','ixz','iyz'),(*inertia,0,0,0))))
        self.links[name]=(s,u)
        if parent:
            jn=name+'_joint';sj=el(self.m,'joint',name=jn,type='revolute' if jtype=='continuous' else jtype)
            el(sj,'parent',parent);el(sj,'child',name);el(sj,'pose','0 0 0 0 0 0',relative_to=name)
            if u is not None:
                uj=el(self.u,'joint',name=jn,type=jtype);el(uj,'parent',link=parent);el(uj,'child',link=name)
                pp=self.poses[parent];el(uj,'origin',xyz=nums([pos[k]-pp[k] for k in range(3)]),rpy=nums(rpy));
            if jtype!='fixed':
                ax=el(sj,'axis');el(ax,'xyz',nums(axis));lim=el(ax,'limit')
                lo,hi=limits or (-1e16,1e16)
                el(lim,'lower',lo);el(lim,'upper',hi);el(lim,'effort',20);el(lim,'velocity',1000)
                dyn=el(ax,'dynamics');el(dyn,'damping',(spring or {}).get('damping',.002 if jtype=='continuous' else .05))
                if spring:
                    for k,v in spring.items():
                        if k!='damping':el(dyn,k,v)
                if u is not None:
                    el(uj,'axis',xyz=nums(axis));el(uj,'limit',lower=lo,upper=hi,effort=20,velocity=1000);el(uj,'dynamics',damping=.05)
        return name
    def shape(self,name,kind,dim,pos=(0,0,0),rpy=(0,0,0),color=(.08,.09,.10,1),collision=True,visual=True,urdf_visual=True):
        s,u=self.links[name]
        for typ,enabled in [('visual',visual),('collision',collision)]:
            if not enabled:continue
            sn=el(s,typ,name=f'{typ}_{len(s)}');el(sn,'pose',nums((*pos,*rpy)));g=el(sn,'geometry');sh=el(g,kind)
            if kind=='box':el(sh,'size',nums(dim))
            elif kind=='cylinder':el(sh,'radius',dim[0]);el(sh,'length',dim[1])
            elif kind=='mesh':el(sh,'uri',str(self.mesh_dir/dim))
            if typ=='visual':
                mat=el(sn,'material');el(mat,'ambient',nums(color));el(mat,'diffuse',nums(color))
            else:
                sf=el(sn,'surface');fr=el(sf,'friction');ode=el(fr,'ode');el(ode,'mu',.9);el(ode,'mu2',.7)
            if u is not None and (typ!='visual' or urdf_visual):
                un=el(u,typ);el(un,'origin',xyz=nums(pos),rpy=nums(rpy));ug=el(un,'geometry')
                if kind=='box':el(ug,kind,size=nums(dim))
                elif kind=='cylinder':el(ug,kind,radius=dim[0],length=dim[1])
                else:el(ug,kind,filename='package://rc_tank_sim/meshes/'+dim)
                if typ=='visual':mat=el(un,'material',name=f'material_{len(u)}');el(mat,'color',rgba=nums(color))
    def plugin(self,filename,name):return el(self.m,'plugin',filename=filename,name=name)
    def build(self):
        c=self.c;heavy=self.profile=='detailed';r=c['sprocket_radius'];a=c['half_axle_spacing'];sep=c['track_separation'];w=c['track_width']
        self.link('base_link',(0,0,0),c['body_mass'],(.53,.287,.093))
        self.shape('base_link','box',(.53,.275,.086),pos=(0,0,.0035),visual=False)
        for part in ('body_front','body_back','cover_front','cover_back'):self.shape('base_link','mesh',part+'.stl',collision=False,urdf_visual=self.visual_detail=='stl')
        if self.visual_detail=='simple':
            visual=el(self.links['base_link'][1],'visual');geometry=el(visual,'geometry')
            el(geometry,'mesh',filename='package://rc_tank_sim/meshes/body_proxy.stl')
            material=el(visual,'material',name='body_proxy');el(material,'color',rgba='.12 .14 .17 1')
        for side,sign in [('left',1),('right',-1)]:
            y=sign*sep/2
            if not heavy:
                n=self.link(side+'_belt',(0,y,-.05),.3,(2*a+2*r,w,2*r),'base_link')
                self.shape(n,'box',(2*a,w,2*r))
                for x in (-a,a):self.shape(n,'cylinder',(r,w),pos=(x,0,0),rpy=(PI/2,0,0))
                p=self.plugin('gz-sim-track-controller-system','gz::sim::systems::TrackController')
                for k,v in {'link':n,'velocity_topic':f'/tank/{side}/track_speed','max_command_age':c['command_timeout'],'min_velocity':-7,'max_velocity':7}.items():el(p,k,v)
            for i,x in enumerate((.155,0,-.155)):
                carrier=self.link(f'{side}_suspension_{i}',(x,y,-.06037),.09,(.06,.016,.015),'base_link',jtype='prismatic' if heavy else 'fixed',axis=(0,0,1),limits=(-.011,.011),spring={'spring_reference':.004,'spring_stiffness':c['suspension_stiffness'],'damping':c['suspension_damping']} if heavy else None)
                self.shape(carrier,'box',(.055,.016,.012),collision=False,color=(.3,.3,.32,1))
                bogie=self.link(f'{side}_bogie_{i}',(x,y,-.06037),.06,(.054,.018,.012),carrier,jtype='revolute' if heavy else 'fixed',limits=(-.4,.4))
                self.shape(bogie,'box',(.054,.018,.012),collision=False,color=(.4,.4,.4,1))
                for j,dx in enumerate((-.027,.027)):
                    n=self.link(f'{side}_road_{i}_{j}',(x+dx,y,-.06037),.055,(.045,.0244,.045),bogie,'continuous')
                    self.shape(n,'cylinder',(.0225,.0244),rpy=(PI/2,0,0),collision=heavy,color=(.18,.18,.18,1))
            for kind,x in [('sprocket',-a),('idler',a)]:
                parent='base_link'
                if heavy and kind=='idler':
                    parent=self.link(side+'_tensioner',(x,y,-.05),.05,(.02,.02,.02),'base_link','prismatic',axis=(1,0,0),limits=(-.005,.012),spring={'spring_reference':.002,'spring_stiffness':2200,'damping':20})
                n=self.link(f'{side}_{kind}',(x,y,-.05),.12,(2*r,.04,2*r),parent,'continuous')
                self.shape(n,'cylinder',(r-.0025,.04),rpy=(PI/2,0,0),collision=heavy,color=(.24,.24,.26,1))
                if heavy:
                    for dy in (-.028,.028):self.shape(n,'cylinder',(r+.003,.003),pos=(0,dy,0),rpy=(PI/2,0,0),color=(.16,.16,.18,1))
            # Return support rollers measured from STL (30 mm diameter).
            for i,x in enumerate((.12,-.12)):
                n=self.link(f'{side}_return_{i}',(x,y,-.03387),.035,(.03,.0214,.03),'base_link','continuous')
                self.shape(n,'cylinder',(.015,.0214),rpy=(PI/2,0,0),collision=heavy)
            if heavy:
                for i,(x,z,t) in enumerate(belt_path(c)):
                    n=self.link(f'{side}_shoe_{i}',(x,y,z),c['shoe_mass'],(.0095,w,.004),urdf=False,rpy=(0,t,0))
                    self.shape(n,'box',(.0095,w,.004),color=(.045,.045,.05,1))
                # Open native hinge chain: 106 revolute joints, one compliant closure per side.
                # Joint frames are at the midpoint between adjacent shoe centres, in model coordinates.
                path=belt_path(c)
                for i in range(c['links_per_side']-1):
                    x0,z0,_=path[i];x1,z1,_=path[i+1]
                    joint=el(self.m,'joint',name=f'{side}_pin_{i}',type='revolute')
                    el(joint,'parent',f'{side}_shoe_{i}');el(joint,'child',f'{side}_shoe_{i+1}')
                    el(joint,'pose',nums(((x0+x1)/2,y,(z0+z1)/2,0,0,0)),relative_to='__model__')
                    axis=el(joint,'axis');el(axis,'xyz','0 1 0');limit=el(axis,'limit')
                    el(limit,'lower',-1.2);el(limit,'upper',1.2)
                    dynamics=el(axis,'dynamics');el(dynamics,'damping',.00002)
        if not heavy:
            p=self.plugin('gz-sim-tracked-vehicle-system','gz::sim::systems::TrackedVehicle')
            for side in ('left','right'):
                tr=el(p,side+'_track');el(tr,'link',side+'_belt');el(tr,'velocity_topic',f'/tank/{side}/track_speed')
            for key,val in {'tracks_separation':sep,'track_height':2*r,'steering_efficiency':1.0,'topic':'/tank/drive_internal','odom_topic':'/tank/unused_track_odom','tf_topic':'/tank/unused_track_tf'}.items():el(p,key,val)
        # Shift servo is visual; effective gear ratio and torque are in dynamics plugin.
        n=self.link('shift_servo',(0,0,.025),.02,(.02,.025,.012),'base_link','revolute',axis=(0,0,1),limits=(0,PI))
        self.shape(n,'box',(.025,.006,.006),pos=(.0125,0,0),collision=False,color=(.8,.4,.1,1))
        self.link('stereo_mount',(c['camera_x'],0,c['camera_z']),.075,(.025,.095,.025),'base_link')
        self.shape('stereo_mount','box',(.025,.095,.025),color=(.6,.62,.65,1))
        for side,sign in [('left',1),('right',-1)]:
            n=self.link('camera_'+side+'_frame',(c['camera_x']+.013,sign*c['stereo_baseline']/2,c['camera_z']),.001,(.002,.002,.002),'base_link')
            # optical axes: z forward, x right, y down (REP-103).
            self.link('camera_'+side+'_optical_frame',self.poses[n],.0001,(.001,.001,.001),n,rpy=(-PI/2,0,-PI/2))
            self.shape(n,'cylinder',(.008,.002),rpy=(0,PI/2,0),collision=False,color=(.01,.01,.015,1))
            if self.sensors and self.cameras:
                sn=el(self.links[n][0],'sensor',name='stereo_'+side,type='camera');el(sn,'always_on','true');el(sn,'update_rate',c['camera_rate']);el(sn,'topic',f'/camera/{side}/image_raw');el(sn,'gz_frame_id','camera_'+side+'_optical_frame')
                cam=el(sn,'camera');el(cam,'horizontal_fov',c['camera_hfov']);img=el(cam,'image');el(img,'width',c['camera_width']);el(img,'height',c['camera_height']);el(img,'format','R8G8B8');clip=el(cam,'clip');el(clip,'near',.05);el(clip,'far',20);el(cam,'camera_info_topic',f'/camera/{side}/camera_info')
                # Projection baseline is set by ROS CameraInfo relay.
        n=self.link('lidar_link',(c['lidar_x'],0,c['lidar_z']),.12,(.06,.06,.035),'base_link')
        self.shape(n,'cylinder',(.03,.035),color=(.1,.1,.1,1))
        self.shape(n,'box',(.022,.022,.027),pos=(0,0,-.031),color=(.3,.3,.3,1))
        if self.sensors and self.lidar:
            sn=el(self.links[n][0],'sensor',name='lidar',type='gpu_lidar');el(sn,'always_on','true');el(sn,'update_rate',c['lidar_rate']);el(sn,'topic','/scan');el(sn,'gz_frame_id','lidar_link');el(sn,'visualize','true')
            ray=el(sn,'lidar');scan=el(ray,'scan');h=el(scan,'horizontal');
            for k,v in {'samples':c['lidar_samples'],'resolution':1,'min_angle':-PI,'max_angle':PI-2*PI/c['lidar_samples']}.items():el(h,k,v)
            ran=el(ray,'range');
            for k,v in {'min':.12,'max':c['lidar_range'],'resolution':.01}.items():el(ran,k,v)
        # 18 front + 3 rear-left + 3 rear-right addressable LEDs from source sketch.
        for i in range(24):
            if i<18:pos=(.269,.1275-i*.015,.009)
            else:pos=(-.266,(.105 if i<21 else -.105)+(i%3-1)*.012,.009)
            self.shape('base_link','box',(.003,.009,.007),pos=pos,collision=False,color=(.02,.02,.02,1))
            self.links['base_link'][0][-1].set('name',f'led_{i}')
        p=self.plugin('libtank_dynamics.so','rc_tank::Dynamics')
        el(p,'detailed',str(heavy).lower())
        for k in ('sprocket_radius','track_separation','links_per_side','hinge_stiffness','hinge_damping','gear_low','gear_high','gear_efficiency','command_timeout','motor_power_w','motor_rpm','half_axle_spacing'):el(p,k,c[k])
        # True chassis odometry rather than a second, inconsistent track odometry TF.
        p=self.plugin('gz-sim-odometry-publisher-system','gz::sim::systems::OdometryPublisher')
        for k,v in {'odom_frame':'odom','robot_base_frame':'base_link','odom_topic':'/tank/odom','tf_topic':'/tank/odom_tf','dimensions':3,'odom_publish_frequency':30}.items():el(p,k,v)
        p=self.plugin('gz-sim-joint-state-publisher-system','gz::sim::systems::JointStatePublisher');el(p,'topic','/tank/joint_states');el(p,'update_rate',20)
        for joint in self.u.findall('joint'):
            if joint.attrib['type']!='fixed':el(p,'joint_name',joint.attrib['name'])
        return self.u,self.s

def generate(config,profile,mesh_dir,out,sensors=True,visual_detail="simple",cameras=True,lidar=True):
    if profile not in ('light','detailed'):raise ValueError('profile must be light or detailed')
    if visual_detail not in ('simple','stl'):raise ValueError('visual_detail must be simple or stl')
    c=json.loads(Path(config).read_text());validate(c)
    u,s=Builder(c,profile,mesh_dir,sensors,visual_detail,cameras,lidar).build();out=Path(out);out.mkdir(parents=True,exist_ok=True)
    for root,name in [(u,'tank.urdf'),(s,'tank.sdf')]:E.indent(root);E.ElementTree(root).write(out/name,encoding='unicode',xml_declaration=True)
    return out/'tank.urdf',out/'tank.sdf'

def validate(c):
    for k,v in c.items():
        if not isinstance(v,(int,float)) or not math.isfinite(v):raise ValueError(f'{k}: finite numeric value required')
    for k in ('motor_rpm','motor_power_w','gear_low','gear_high','sprocket_radius','half_axle_spacing','track_separation','track_width','body_mass','shoe_mass','hinge_stiffness','hinge_damping','suspension_stiffness','suspension_damping','camera_width','camera_height','camera_rate','lidar_rate','lidar_range','command_timeout','stereo_baseline'):
        if c[k]<=0:raise ValueError(f'{k} must be positive')
    for k in ('links_per_side','camera_width','camera_height','lidar_samples'):
        if int(c[k])!=c[k] or c[k]<2:raise ValueError(f'{k}: integer >= 2 required')
    if c['gear_low']<=c['gear_high']:raise ValueError('low ratio must exceed high ratio')
    if c['lidar_range']<=.12:raise ValueError('lidar_range must exceed near limit 0.12 m')
    if not 0<c['gear_efficiency']<=1:raise ValueError('gear_efficiency must be in (0,1]')
    if not 0<c['camera_hfov']<PI:raise ValueError('invalid horizontal FOV')

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--config',required=True);ap.add_argument('--profile',choices=['light','detailed'],default='light');ap.add_argument('--mesh-dir',required=True);ap.add_argument('--out',required=True);ap.add_argument('--no-sensors',action='store_true');a=ap.parse_args();generate(a.config,a.profile,a.mesh_dir,a.out,not a.no_sensors)
