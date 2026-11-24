"""Generate local Gazebo assets; no downloads, no changes to reference URDF."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as E
from scenarios import layout


def box(parent, name, pose, size, static=True):
    model=E.SubElement(parent,'model',name=name)
    E.SubElement(model,'static').text=str(static).lower()
    E.SubElement(model,'pose').text=' '.join(map(str,pose))
    link=E.SubElement(model,'link',name='body')
    for kind in ('collision','visual'):
        geom=E.SubElement(E.SubElement(link,kind,name=kind),'geometry')
        E.SubElement(E.SubElement(geom,'box'),'size').text=' '.join(map(str,size))
    return model


def robot_urdf(source, mesh_root, name):
    root=E.parse(source).getroot();root.set('name',name)
    colors={'robot_a':'0.337 0.706 1 1','robot_b':'1 0.706 0.329 1',
            'robot_c':'0.427 0.867 0.608 1','robot_d':'0.851 0.569 1 1'}
    for color in root.findall("material[@name='white']/color"):
        color.set('rgba',colors[name])
    for control in list(root.findall('ros2_control')):root.remove(control)
    for mesh in root.findall('.//mesh'):
        path=mesh_root/Path(mesh.get('filename')).name
        if not path.is_file():raise ValueError(f'Missing mesh: {path}')
        mesh.set('filename','file://'+str(path))
    # Reference body has no collision; wheels already have cylinder collisions.
    base=root.find("link[@name='base_link']")
    if base.find('collision') is None:
        coll=E.SubElement(base,'collision')
        E.SubElement(coll,'origin',xyz='0 0 0.32',rpy='0 0 0')
        E.SubElement(E.SubElement(coll,'geometry'),'box',size='0.70 0.60 0.40')
    for joint in root.findall('joint'):
        if joint.get('name','').startswith('steering_'):
            joint.find('limit').set('lower','-0.959');joint.find('limit').set('upper','0.959')
    for link in root.findall('link'):
        if link.get('name','').endswith('_wheel_link'):
            gazebo=E.SubElement(root,'gazebo',reference=link.get('name'))
            E.SubElement(gazebo,'mu1').text='1.0';E.SubElement(gazebo,'mu2').text='1.0'
    plugin=E.SubElement(E.SubElement(root,'gazebo'),'plugin',
                        filename='libantbot_gazebo_io.so',name='antbot_gazebo::RobotIO')
    E.SubElement(plugin,'robot_namespace').text=name
    return root


def generate(reference, output, scenario, count, check=False):
    scene=layout(scenario,count);output.mkdir(parents=True,exist_ok=True)
    sdf=E.Element('sdf',version='1.9');world=E.SubElement(sdf,'world',name='antbot_world')
    physics=E.SubElement(world,'physics',name='physics',type='ignored')
    E.SubElement(physics,'max_step_size').text='0.005'
    E.SubElement(physics,'real_time_factor').text='1.0'
    E.SubElement(world,'gravity').text='0 0 -9.81'
    for filename,name in [('gz-sim-physics-system','Physics'),('gz-sim-user-commands-system','UserCommands'),('gz-sim-scene-broadcaster-system','SceneBroadcaster')]:
        E.SubElement(world,'plugin',filename=filename,name='gz::sim::systems::'+name)
    light=E.SubElement(world,'light',name='sun',type='directional')
    E.SubElement(light,'pose').text='0 0 10 0 0 0';E.SubElement(light,'direction').text='-0.5 0.2 -1'
    E.SubElement(light,'diffuse').text='0.8 0.8 0.8 1'
    box(world,'ground',(0,0,-.05,0,0,0),(40,40,.1))
    if scene.bounds:
        xmin,xmax,ymin,ymax=scene.bounds
        for name,x,y,sx,sy in [('south',0,ymin-.05,xmax-xmin+.2,.1),('north',0,ymax+.05,xmax-xmin+.2,.1),
                              ('west',xmin-.05,0,.1,ymax-ymin),('east',xmax+.05,0,.1,ymax-ymin)]:
            box(world,name,(x,y,.5,0,0,0),(sx,sy,1))
    for i,(x,y,radius) in enumerate(scene.obstacles):
        model=E.SubElement(world,'model',name=f'obstacle_{i}');E.SubElement(model,'static').text='true'
        E.SubElement(model,'pose').text=f'{x} {y} .5 0 0 0';link=E.SubElement(model,'link',name='body')
        for kind in ('collision','visual'):
            cylinder=E.SubElement(E.SubElement(E.SubElement(link,kind,name=kind),'geometry'),'cylinder')
            E.SubElement(cylinder,'radius').text=str(radius);E.SubElement(cylinder,'length').text='1'
        # High-contrast visual; collision geometry is unchanged.
        material=E.SubElement(link.find('visual'),'material')
        E.SubElement(material,'ambient').text='0.8 0.025 0.015 1'
        E.SubElement(material,'diffuse').text='1 0.05 0.025 1'
        E.SubElement(material,'emissive').text='0.22 0.008 0.004 1'

    for name,config in scene.robots.items():
        meshes=reference/'robot_description/meshes'
        if not meshes.is_dir():meshes=Path(__file__).resolve().parents[1]/'assets/meshes'
        root=robot_urdf(reference/'robot_model/antbot.urdf',meshes,name)
        plugin=root.find('gazebo/plugin')
        E.SubElement(plugin,'scenario').text=scenario
        E.SubElement(plugin,'robot_count').text=str(count)
        path=output/f'{name}.urdf';E.ElementTree(root).write(path,encoding='unicode')
        if not check:
            result=subprocess.run(['gz','sdf','-p',str(path)],check=True,capture_output=True,text=True)
            model=E.fromstring(result.stdout).find('model')
            if model is None:raise RuntimeError('URDF conversion returned no model')
            pose=model.find('pose')
            if pose is None:pose=E.SubElement(model,'pose')
            x,y,yaw=config['start'];pose.text=f'{x} {y} 0.01 0 0 {yaw}'
            world.append(model)
    E.indent(sdf);E.ElementTree(sdf).write(output/'world.sdf',encoding='unicode')
    (output/'scene.json').write_text(json.dumps(dict(scenario=scenario,count=count,robots=scene.robots,bounds=scene.bounds,obstacles=scene.obstacles),indent=2))
    return scene

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--reference',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--scenario',choices=list('ABCD'),default='B');p.add_argument('--robots',type=int,choices=[3,4],default=3)
    p.add_argument('--check',action='store_true');a=p.parse_args()
    generate(a.reference,a.output,a.scenario,a.robots,a.check)
    print(f'{a.scenario}: {a.robots} robots; assets: {a.output}; URDF conversion={not a.check}')
