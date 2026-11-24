"""Generate short, forward-only B crossing references without terminal loops."""
import json
import math
from pathlib import Path


def generate():
    folder=Path(__file__).resolve().parents[1]/'references'
    source=json.loads((folder/'B_recorded.json').read_text())
    duration=20.;lane=1.;step=.02
    endpoints={'robot_a':((-3.,0.),(3.,0.),0.),
               'robot_b':((0.,-3.),(0.,3.),math.pi/2),
               'robot_c':((3.,0.),(-3.,0.),math.pi),
               'robot_d':((0.,3.),(0.,-3.),-math.pi/2)}
    robots={}
    for robot,(start,goal,goal_yaw) in endpoints.items():
        dx,dy=goal[0]-start[0],goal[1]-start[1];length=math.hypot(dx,dy)
        nx,ny=dy/length,-dx/length
        rows=[]
        for i in range(round((duration+2.)/step)+1):
            t=i*step;u=min(1.,t/duration)
            s=u**3*(10.-15.*u+6.*u*u)
            rate=30.*u*u*(1.-u)**2/duration if u<1 else 0.
            offset=lane*math.sin(math.pi*s)**2
            derivative=lane*math.pi*math.sin(2.*math.pi*s)
            second=2.*lane*math.pi**2*math.cos(2.*math.pi*s)
            vx,vy=dx+nx*derivative,dy+ny*derivative
            px,py=start[0]+dx*s+nx*offset,start[1]+dy*s+ny*offset
            yaw=math.atan2(vy,vx)
            speed=rate*math.hypot(vx,vy)
            angular=rate*(vx*ny*second-vy*nx*second)/(vx*vx+vy*vy)
            if u>=1.:px,py=goal;yaw=goal_yaw
            rows.append([round(t,8),px,py,yaw,speed,angular])
        robots[robot]=rows
    data=dict(version=1,scenario='B',frame='world',reference_kind='generated smooth crossing routes',
              source_result='generated_reference',original_source_bag=source.get('source_bag'),
              columns=['t','x','y','yaw','v','w'],duration_seconds=duration,
              lateral_offset_m=lane,robots=robots,
              note='Direct start-to-goal progress with a small right-side bend at the central crossing. No reversal, overshoot or terminal loops. Ego still plans with MPC; only fixed followers execute these references.')
    target=folder/'B.json';target.write_text(json.dumps(data,separators=(',',':')))
    print(target)


if __name__=='__main__':generate()
