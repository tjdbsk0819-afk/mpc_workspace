"""Build a fixed overhead Gazebo view and a measured-speed HUD."""
import json
import math
import os
from pathlib import Path
import xml.etree.ElementTree as E
import yaml
from scenarios import layout


def generate_gui(reference, metadata=None):
    data=json.loads(metadata.read_text()) if metadata and metadata.exists() else None
    if data:
        scene=layout(data['scenario'],data['count'])
    else:
        scene=layout('B',3)
    points=[pose[:2] for item in scene.robots.values() for pose in (item['start'],item['goal'])]
    xs,ys=zip(*points);cx=(min(xs)+max(xs))/2;cy=(min(ys)+max(ys))/2
    width=max(xs)-min(xs)+2.;height=max(ys)-min(ys)+2.
    altitude=max(6.,width*.62,height*.95)
    backend=os.environ.get('ANTBOT_PLANNER','mpc')
    cfg=reference/('mpc_planning/mpc_planner_antbot/config/settings.yaml' if backend=='mpc' else 'controller_bringup/config/dmpc.yaml')
    values=yaml.safe_load(cfg.read_text())
    speed=values['antbot_limits']['speed'] if backend=='mpc' else values['/**']['ros__parameters']['max_speed']
    root=E.Element('gui_config')
    E.SubElement(root,'dialog',name='quick_start',show_again='false')
    window=E.SubElement(root,'window');E.SubElement(window,'width').text='1280';E.SubElement(window,'height').text='820'
    E.SubElement(window,'dialog_on_exit').text='false'
    E.SubElement(window,'style',material_theme='Dark',material_primary='BlueGrey',material_accent='LightBlue')
    def plugin(filename,name,state='floating',width=5,height=5):
        p=E.SubElement(root,'plugin',filename=filename,name=name);g=E.SubElement(p,'gz-gui')
        E.SubElement(g,'title').text=name
        for key,kind,val in [('state','string',state),('showTitleBar','bool','false'),('resizable','bool','false'),('width','double',str(width)),('height','double',str(height))]:
            E.SubElement(g,'property',key=key,type=kind).text=val
        return p,g
    view,g=plugin('MinimalScene','3D View','docked',1000,750)
    E.SubElement(view,'engine').text='ogre';E.SubElement(view,'scene').text='scene'
    E.SubElement(view,'ambient_light').text='.6 .6 .6'
    E.SubElement(view,'background_color').text='.16 .20 .26'
    E.SubElement(view,'camera_pose').text=f'{cx} {cy} {altitude} 0 {math.pi/2} {math.pi/2}'
    E.SubElement(view,'anti_aliasing').text='0'
    plugin('GzSceneManager','Scene Manager')
    plugin('MarkerManager','Path markers')
    # No InteractiveViewControl or CameraTracking: drag/scroll cannot move the camera.
    # Compact horizontal HUD stays above the horizontal robot lanes.
    panel_width=16+104*len(scene.robots)+6*(len(scene.robots)-1)
    panel,g=plugin('RobotSpeedPanel','Robot speeds',width=panel_width,height=108)
    E.SubElement(g,'property',key='z',type='double').text='10'
    anchors=E.SubElement(g,'anchors',target='3D View')
    for side in ('left','top'):E.SubElement(anchors,'line',own=side,target=side)
    E.SubElement(panel,'robots').text=' '.join(scene.robots)
    E.SubElement(panel,'max_speed').text=str(speed)
    for robot,item in scene.robots.items():
        x,y,yaw=item['goal'];E.SubElement(panel,'goal',robot=robot,x=str(x),y=str(y),yaw=str(yaw))
    control,g=plugin('WorldControl','World control',width=210,height=72)
    anchors=E.SubElement(g,'anchors',target='3D View')
    E.SubElement(anchors,'line',own='left',target='left');E.SubElement(anchors,'line',own='bottom',target='bottom')
    for key,value in [('play_pause','true'),('step','true'),('start_paused','false'),('use_event','true')]:E.SubElement(control,key).text=value
    E.indent(root)
    # gz-gui configs intentionally contain multiple top-level XML elements.
    return '\n'.join(E.tostring(child,encoding='unicode') for child in root)

if __name__=='__main__':
    output=Path('/tmp/antbot-gazebo-fixed.config')
    output.write_text(generate_gui(Path(os.environ['ANTBOT_REFERENCE']),Path('/workspace/gazebo/generated/scene.json')))
    print(output)
