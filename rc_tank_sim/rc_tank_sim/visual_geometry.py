"""Batched marker geometry, independent of ROS; no numpy at runtime."""
import math
CORNERS=((-1,-1,-1),(-1,-1,1),(-1,1,-1),(-1,1,1),(1,-1,-1),(1,-1,1),(1,1,-1),(1,1,1))
FACES=((0,3,1),(0,2,3),(4,5,7),(4,7,6),(0,1,5),(0,5,4),(2,6,7),(2,7,3),(0,4,6),(0,6,2),(1,3,7),(1,7,5))

def shoe_triangles(position,quaternion,width):
    if not all(math.isfinite(v) for v in (*position,*quaternion,width)):return []
    px,py,pz=position;x,y,z,w=quaternion
    norm=math.sqrt(x*x+y*y+z*z+w*w)
    if norm<1e-9:return []
    x,y,z,w=x/norm,y/norm,z/norm,w/norm
    vertices=[]
    for a,b,c in CORNERS:
        vx,vy,vz=a*.0095/2,b*width/2,c*.004/2
        tx,ty,tz=2*(y*vz-z*vy),2*(z*vx-x*vz),2*(x*vy-y*vx)
        vertices.append((px+vx+w*tx+y*tz-z*ty,py+vy+w*ty+z*tx-x*tz,pz+vz+w*tz+x*ty-y*tx))
    return [vertices[i] for f in FACES for i in f]

def scan_segments(ranges,angle_min,angle_increment,range_min,range_max):
    """Only measured finite returns, never fabricated max-range detections."""
    result=[]
    for i,r in enumerate(ranges):
        if not math.isfinite(r) or not range_min<=r<=range_max:continue
        a=angle_min+i*angle_increment
        result.append(((0.,0.,0.),(r*math.cos(a),r*math.sin(a),0.)))
    return result
