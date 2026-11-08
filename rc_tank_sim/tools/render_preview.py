#!/usr/bin/env python3
"""Engineering preview of generated SDF visuals, not a Gazebo runtime screenshot."""
import math,struct,sys
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from rc_tank_sim.model import generate

def matrix(pose):
 p=list(map(float,pose.split())) if pose else [0]*6
 x,y,z,r,pitch,yaw=p;cr,sr=math.cos(r),math.sin(r);cp,sp=math.cos(pitch),math.sin(pitch);cy,sy=math.cos(yaw),math.sin(yaw)
 return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]),np.array([x,y,z])

def faces(g):
 if g.find('mesh') is not None:
  p=Path(g.findtext('mesh/uri'));b=p.read_bytes();n=struct.unpack_from('<I',b,80)[0]
  return np.ndarray((n,),dtype=[('n','<f4',3),('v','<f4',(3,3)),('a','<u2')],buffer=b,offset=84)['v'].copy()
 if g.find('box') is not None:
  s=np.array(list(map(float,g.findtext('box/size').split())))/2
  v=np.array([[-1,-1,-1],[-1,-1,1],[-1,1,-1],[-1,1,1],[1,-1,-1],[1,-1,1],[1,1,-1],[1,1,1]])*s
  return v[np.array([[0,1,3,2],[4,6,7,5],[0,4,5,1],[2,3,7,6],[0,2,6,4],[1,5,7,3]])]
 if g.find('cylinder') is not None:
  r=float(g.findtext('cylinder/radius'));h=float(g.findtext('cylinder/length'))/2
  a=np.arange(20)*2*np.pi/20;v=np.column_stack((r*np.cos(a),r*np.sin(a),np.ones(20)*h));v2=v.copy();v2[:,2]=-h
  return [v,v2]+[np.array([v[i],v[(i+1)%20],v2[(i+1)%20],v2[i]]) for i in range(20)]
 return []
fig=plt.figure(figsize=(13,7.5),facecolor='#eef1f4')
for i,mode in enumerate(('light','detailed')):
 _,s=generate(ROOT/'config/robot.json',mode,ROOT/'meshes',ROOT.parent/'preview_models'/mode)
 ax=fig.add_subplot(1,2,i+1,projection='3d',facecolor='#eef1f4')
 model=E.parse(s).getroot().find('model');all_polys=[];all_colors=[]
 for link in model.findall('link'):
  R,t=matrix(link.findtext('pose'))
  for vis in link.findall('visual'):
   rr,tt=matrix(vis.findtext('pose'));f=faces(vis.find('geometry'))
   polys=[(R@(rr@x.T+tt[:,None])+t[:,None]).T for x in f]
   c=list(map(float,vis.findtext('material/diffuse').split()))
   if vis.find('geometry/mesh') is not None:c=[.18,.20,.23,1]
   all_polys.extend(polys);all_colors.extend([c]*len(polys))
 ax.add_collection3d(Poly3DCollection(all_polys,facecolors=all_colors,edgecolor=(.1,.12,.15,.12),linewidth=.12,rasterized=True))
 ax.set(xlim=(-.31,.31),ylim=(-.23,.23),zlim=(-.10,.16));ax.set_box_aspect((.62,.46,.26));ax.view_init(elev=24,azim=38)
 ax.set_axis_off();ax.set_title('LIGHT · surface-motion tracks' if mode=='light' else 'DETAILED · 214 physical shoes',fontsize=13,color='#243348',pad=0)
fig.suptitle('RC TANK · ROS 2 JAZZY / GAZEBO HARMONIC',fontsize=18,color='#243348',y=.93)
fig.text(.5,.08,'Front stereo camera · Top-mounted 360° lidar · Reduced STL body\nGeometry preview only — not a Gazebo runtime capture',ha='center',fontsize=11,color='#526075')
fig.subplots_adjust(left=0,right=1,bottom=.13,top=.87,wspace=-.13)
fig.savefig(ROOT/'docs/model_preview.png',dpi=150)
