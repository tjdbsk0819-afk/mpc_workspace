"""Differential regression: native MPC/RPP sampling and guard vs previous Python."""
from pathlib import Path
import math,random,subprocess,sys,tempfile
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'controller_bringup'))
from antbot_mpc_ros.timed_tracking import Motion,TimedTrajectory,command_clearance,filter_command

DRIVER=r'''
#include "mpc_tracking.hpp"
#include <iomanip>
#include <iostream>
int main(){using namespace antbot_tracking;std::cout<<std::setprecision(17);char op;
while(std::cin>>op){
 if(op=='S'){Trajectory t;int n;double seconds;std::cin>>n>>seconds;
  for(int k=0;k<n;++k){double time,x,y,yaw,v,a,w;std::cin>>time>>x>>y>>yaw>>v>>a>>w;
   t.times.push_back(time);t.poses.push_back({x,y,yaw});t.velocities.push_back(v);t.accelerations.push_back(a);t.rates.push_back(w);}
  t.validate();auto s=t.sample(seconds);std::cout<<s.x<<' '<<s.y<<' '<<s.yaw<<' '<<s.v<<' '<<s.w<<'\n';
 }else{Motion s;double v,w,hold,accel,yaccel,brake;std::cin>>s.x>>s.y>>s.yaw>>s.v>>s.w>>v>>w>>hold>>accel>>yaccel>>brake;
  if(op=='C'){Neighbor p;std::cin>>p.x>>p.y>>p.vx>>p.vy;p.separation=1.15;std::cout<<clearance(s,v,w,p,hold,accel,yaccel,brake)<<'\n';}
  else{int n;double mpc,rpp;std::cin>>n>>mpc>>rpp;std::vector<Neighbor> peers;
   for(int i=0;i<n;++i){Neighbor p;std::cin>>p.x>>p.y>>p.vx>>p.vy>>p.separation;peers.push_back(p);}
   auto out=filter_command(s,v,w,peers,hold,accel,yaccel,brake,mpc,rpp);
   if(!out)std::cout<<"NONE\n";else std::cout<<out->v<<' '<<out->w<<' '<<out->clearance<<' '<<out->scale<<'\n';}
 }}
 ReleaseGate g{150000000,100000000};g.stop();if(g.allow(0)||g.allow(100000000)||!g.allow(150000000))return 2;
 g.stop();if(g.allow(200000000)||g.allow(400000000)||g.allow(500000000)||!g.allow(550000000))return 3;
}
'''

def main():
    rng=random.Random(2691);lines=[];expected=[]
    for _ in range(80):
        times=tuple(k*.2 for k in range(6));poses=tuple((rng.uniform(-2,2),rng.uniform(-2,2),rng.uniform(-3,3)) for _ in times)
        vs=tuple(rng.uniform(0,1) for _ in times);acc=tuple(rng.uniform(-.5,.5) for _ in times);rates=tuple(rng.uniform(-.8,.8) for _ in times)
        seconds=rng.uniform(0,1);t=TimedTrajectory(times,poses,vs,acc,rates);p=t.sample(seconds)
        values=[6,seconds]
        for k in range(6):values.extend([times[k],*poses[k],vs[k],acc[k],rates[k]])
        lines.append('S '+' '.join(map(str,values)));expected.append((p.x,p.y,p.yaw,p.v,p.w))
    for _ in range(80):
        s=Motion(rng.uniform(-1,1),rng.uniform(-1,1),rng.uniform(-3,3),rng.uniform(0,1),rng.uniform(-.8,.8))
        v,w=rng.uniform(0,1),rng.uniform(-.8,.8);peer=(rng.uniform(-3,3),rng.uniform(-3,3),rng.uniform(-1,1),rng.uniform(-1,1))
        values=[s.x,s.y,s.yaw,s.v,s.w,v,w,.25,.5,.5,.5,*peer]
        lines.append('C '+' '.join(map(str,values)))
        expected.append((command_clearance(s,v,w,peer,.25,.5,.5,.5),))
    # Random multi-neighbor commands plus the previously fixed close-rotation case.
    cases=[(Motion(0,0,0,0,0),.1,.2,[((1.155,0,0,0),1.1545)])]
    for _ in range(100):
        cases.append((Motion(0,0,rng.uniform(-3,3),rng.uniform(0,1),rng.uniform(-.5,.5)),
                      rng.uniform(0,1),rng.uniform(-.8,.8),
                      [((rng.uniform(-4,4),rng.uniform(-4,4),rng.uniform(-.3,.3),rng.uniform(-.3,.3)),1.155)]))
    for s,v,w,neighbors in cases:
        values=[s.x,s.y,s.yaw,s.v,s.w,v,w,.25,.5,.5,.5,len(neighbors),w,w]
        for p,d in neighbors:values.extend([*p,d])
        lines.append('F '+' '.join(map(str,values)))
        peer,distance=neighbors[0]
        expected.append(filter_command(s,v,w,peer,distance,.25,.5,.5,.5,mpc_yaw_rate=w,rpp_yaw_rate=w))
    with tempfile.TemporaryDirectory(prefix='antbot-native-parity-') as directory:
        source=Path(directory)/'check.cpp';binary=Path(directory)/'check';source.write_text(DRIVER)
        subprocess.run(['g++','-std=c++17','-O2','-I',str(ROOT/'path_following_controller/src'),
                        '-I',str(ROOT/'ros_messages/antbot_interfaces/include'),str(source),'-o',str(binary)],check=True)
        result=subprocess.run([str(binary)],input='\n'.join(lines)+'\n',text=True,capture_output=True,check=True)
    actual=result.stdout.splitlines();assert len(actual)==len(expected)
    for i,(line,want) in enumerate(zip(actual,expected)):
        if want is None:assert line=='NONE',(i,line,want)
        else:
            got=tuple(map(float,line.split()));assert len(got)==len(want),(i,got,want)
            assert all(math.isclose(a,b,rel_tol=1e-10,abs_tol=1e-10) for a,b in zip(got,want)),(i,got,want)
    print(f'PASS: {len(expected)} native/Python sampling, braking and all-neighbor guard comparisons; release gate checks')

if __name__=='__main__':main()
