#!/usr/bin/env python3
"""Rebuild the RViz convex body proxy from the four reduced STL assets.
Development tool only: numpy + scipy. Runtime simulation uses neither.
"""
import json,struct
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull
root=Path(__file__).resolve().parents[1];points=[]
for name in ('body_front','body_back','cover_front','cover_back'):
    b=(root/'meshes'/f'{name}.stl').read_bytes();n=struct.unpack_from('<I',b,80)[0]
    v=np.ndarray((n,),dtype=[('n','<f4',3),('v','<f4',(3,3)),('a','<u2')],buffer=b,offset=84)['v'].reshape(-1,3);points.extend(v)
v=np.unique(np.round(np.asarray(points),5),axis=0);h=ConvexHull(v);tri=v[h.simplices].copy()
norm=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);flip=(norm*h.equations[:,:3]).sum(1)<0;tri[flip]=tri[flip][:,[0,2,1]]
b=bytearray(b'Convex visual proxy from reduced user STL, metres'.ljust(80,b' '));b+=struct.pack('<I',len(tri))
for t in tri:
    n=np.cross(t[1]-t[0],t[2]-t[0]);n/=np.linalg.norm(n);b+=struct.pack('<12fH',*n,*t.ravel(),0)
(root/'meshes/body_proxy.stl').write_bytes(b)
(root/'docs/proxy_provenance.json').write_text(json.dumps({'method':'convex hull of four reduced source STL parts','input_triangles':8639,'output_triangles':len(tri),'units':'metres','visual_only':True},indent=2))
print(f'Proxy: {len(tri)} triangles')
