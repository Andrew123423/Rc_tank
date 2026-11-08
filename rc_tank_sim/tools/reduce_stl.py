#!/usr/bin/env python3
"""Vertex clustering of supplied print STL parts; numpy only, offline asset tool."""
import argparse,hashlib,json,struct
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser();parser.add_argument('--source',type=Path,default=root.parent/'source');args=parser.parse_args()
source=args.source
manifest=[]
for name,output,mode in [
 ('Main Body - front (1).stl','body_front','body'),
 ('Main Body - back (2).stl','body_back','body'),
 ('Cover - front.stl','cover_front','cover'),
 ('Cover - back.stl','cover_back','body')]:
 b=(source/name).read_bytes(); n=struct.unpack_from('<I',b,80)[0]
 a=np.ndarray((n,),dtype=[('n','<f4',3),('v','<f4',(3,3)),('a','<u2')],buffer=b,offset=84)
 v=a['v'].reshape(-1,3).astype(float)
 # Different print orientations in this STL set. Front cover is laid flat.
 if mode=='body': v=np.column_stack((-v[:,0]-228,v[:,2]-137.5,v[:,1]))
 else: v=np.column_stack((-v[:,0]-228,v[:,1]+137.5,v[:,2]))
 v*=.001
 ids=np.round(v/.0035).astype(int)
 _,inverse=np.unique(ids,axis=0,return_inverse=True)
 count=np.bincount(inverse)
 cent=np.column_stack([np.bincount(inverse,weights=v[:,i])/count for i in range(3)])
 faces=inverse.reshape(-1,3)
 faces=faces[(faces[:,0]!=faces[:,1])&(faces[:,0]!=faces[:,2])&(faces[:,1]!=faces[:,2])]
 # Retain orientation while removing duplicate facets.
 _,idx=np.unique(np.sort(faces,axis=1),axis=0,return_index=True)
 tri=cent[faces[np.sort(idx)]]
 cross=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]); area=np.linalg.norm(cross,axis=1)
 tri=tri[area>1e-12]; cross=cross[area>1e-12]; cross/=np.linalg.norm(cross,axis=1)[:,None]
 out=bytearray(b'Visual-only clustered STL, metres'.ljust(80,b' '));out+=struct.pack('<I',len(tri))
 for t,norm in zip(tri,cross):out+=struct.pack('<12fH',*norm,*t.reshape(-1),0)
 (root/'meshes'/f'{output}.stl').write_bytes(out)
 manifest.append({'source':name,'sha256':hashlib.sha256(b).hexdigest(),'input_triangles':n,'output':f'{output}.stl','output_triangles':len(tri),'units':'metres','cell_size_m':.0035,'visual_only':True})
(root/'docs/mesh_provenance.json').write_text(json.dumps(manifest,indent=2))
print(json.dumps(manifest,indent=2))
