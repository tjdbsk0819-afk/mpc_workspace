"""Display original C++ MPC paths, GT and optional prediction/reference overlays."""
import argparse
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

DEFAULT_ROBOTS = ('robot_a', 'robot_b', 'robot_c')


def topic_identity(topic, robots):
    """Keep prediction points separate from measured odometry, in both layouts."""
    parts = topic.strip('/').split('/')
    if len(parts) < 2 or parts[0] not in robots:
        raise ValueError(f'Unexpected viewer topic: {topic}')
    observer, suffix = parts[0], parts[1:]
    if suffix == ['odom']:
        kind, target = 'odom', observer
    elif suffix == ['mpc_path']:
        kind, target = 'plan', observer
    elif suffix == ['mpc_reference_path']:
        kind, target = 'reference', observer
    elif len(suffix) == 1 and suffix[0].startswith('mpc_candidate_'):
        kind, target = 'candidate', observer
    elif suffix in (['mpc_peer_prediction'], ['peer_prediction']):
        # Legacy two-robot topics do not encode their target in the path.
        target = {'robot_a': 'robot_b', 'robot_b': 'robot_a'}.get(observer)
        kind = 'prediction'
    elif len(suffix) == 2 and suffix[0] == 'mpc_peer_prediction':
        kind, target = 'prediction', suffix[1]
    elif len(suffix) == 3 and suffix[0] == 'peers' and suffix[2] == 'prediction':
        kind, target = 'prediction', suffix[1]
    else:
        raise ValueError(f'Unrecognized viewer topic: {topic}')
    if target not in robots or (kind == 'prediction' and target == observer):
        raise ValueError(f'Invalid prediction target: {topic}')
    return dict(kind=kind, observer=observer, target=target)


PAGE = r'''<!doctype html>
<html lang="ko"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>AntBot MPC Planner 생성 경로</title>
<style>
body{margin:0;background:#101923;color:#e6edf3;font:15px system-ui;display:flex;flex-direction:column;height:100vh}
header{padding:16px 22px;background:#182532}h1{font-size:22px;margin:0 0 10px}
p{margin:6px 0}button,select{cursor:pointer;padding:6px 12px;margin-right:8px}
label{display:inline-block;margin:5px 14px 5px 0}
#status{white-space:pre-line;font:14px system-ui;min-height:42px;line-height:1.6}
canvas{display:block;width:100%;height:0;flex:1;min-height:280px}
.a{color:#56b4ff}.b{color:#ffb454}small{color:#b4c5d5}
</style>
<header><h1 id="title">MPC 생성 경로 · 상대 로봇 CV 예측</h1>
<p id="legend">ROS 토픽 대기 중</p>
<p><small>굵은 실선: 자기 MPC 계획 · 점선: 상대 CV 예측 · 원: 실제 GT 위치 · ×: 계획 끝점 · world [m]</small></p>
<p><select id="focus"><option value="all">전체 로봇 경로</option></select>
<label><input type="checkbox" id="predictions" checked>상대 CV 예측도 표시</label>
<label><input type="checkbox" id="reference">참조 경로도 표시</label>
<label><input type="checkbox" id="candidates">최적화된 MPC 후보도 표시</label></p>
<p><button id="fit">전체 보기</button><button id="in">확대 +</button>
<button id="out">축소 −</button><small>실제 위치가 정지해 있어도 생성된 계획 경로를 볼 수 있습니다.</small></p>
<div id="status">ROS 토픽 대기 중…</div></header><canvas id="view"></canvas>
<script>
const canvas=document.getElementById('view'),ctx=canvas.getContext('2d');
const colors={robot_a:'#56b4ff',robot_b:'#ffb454',robot_c:'#6ddd9b'};
let data={topics:{}},center=[0,0],span=10,autoFit=true,connected=false;
let configuredRobots='';
const shortName=r=>r.replace(/^robot_/,'').toUpperCase();
const descriptions={tracking:'경로 생성 정상',waiting_for_fresh_ego_and_peer:'자기·상대 GT 입력 대기',
 solver_failed:'최적화 실패',insufficient_current_clearance:'현재 로봇 간 여유 거리 부족',
 goal_reached:'목표 도착',solution_expired:'계산 결과 만료',nonfinite_solution:'계산 결과 오류',
 solution_clearance_rejected:'예측 간격 위반으로 경로 제외'};
const controlDescriptions={tracking:'주행 중',tracking_mpc_turn:'MPC 경로의 회전 구간 추종 중',disabled:'주행 꺼짐',proximity_stop:'근거리 정지',
 proximity_recovery_wait:'근거리 정지 해제 확인 중',
 tracking_safe_rotation:'안전 검사 통과 제자리 회전',
 mpc_stop:'MPC 정지',waiting_for_new_plan:'새 계획 대기',waiting_for_fresh_mpc_plan:'MPC 계획 대기',
 waiting_for_fresh_state:'GT 입력 대기',waiting_for_fresh_gt:'GT 입력 대기'};
const recoveryDescriptions={tracking:'기본 계획',tracking_candidate:'후보 재최적화',
 no_feasible_candidate:'실행 가능한 후보 없음',insufficient_current_clearance:'안전거리 부족',
 waiting_for_inputs:'GT 입력 대기',goal_reached:'목표 도착',solver_failed:'최적화 실패',solution_expired:'계획 만료'};
document.getElementById('fit').onclick=()=>{autoFit=true;draw()};
document.getElementById('in').onclick=()=>{autoFit=false;span=Math.max(.2,span/1.4);draw()};
document.getElementById('out').onclick=()=>{autoFit=false;span=Math.min(10000,span*1.4);draw()};
for(const id of ['focus','predictions','reference','candidates'])document.getElementById(id).onchange=()=>{autoFit=true;draw()};
function draw(){
 const robots=data.robots||[];
 document.getElementById('title').textContent=`AntBot ${robots.length||'—'}대 · MPC 생성 경로 · 상대 CV 예측`;
 if(configuredRobots!==robots.join(',')){
  configuredRobots=robots.join(',');
  const select=document.getElementById('focus'),previous=select.value;
  select.replaceChildren(new Option('전체 로봇 경로','all'));
  const legend=document.getElementById('legend');legend.replaceChildren();
  for(const r of robots){
   select.add(new Option(`${shortName(r)}의 MPC·상대 예측만 보기`,r));
   const item=document.createElement('span');item.style.color=colors[r]||'#ffffff';
   item.textContent=`━ ${shortName(r)} MPC / ${shortName(r)}의 미래 위치 점선　`;legend.append(item);
  }
  select.value=robots.includes(previous)?previous:'all';
 }
 const w=canvas.clientWidth,h=canvas.clientHeight,dpr=window.devicePixelRatio||1;
 canvas.width=w*dpr;canvas.height=h*dpr;ctx.setTransform(dpr,0,0,dpr,0,0);
 const focus=document.getElementById('focus').value;
 const rank=v=>({reference:0,prediction:1,candidate:1.5,plan:2,odom:3})[v.kind]??4;
 const entries=Object.entries(data.topics).filter(([,v])=>
  robots.includes(v.observer)&&robots.includes(v.target)&&
  (focus==='all'||v.observer===focus||v.kind==='odom')&&
  (v.kind!=='prediction'||document.getElementById('predictions').checked)&&
  (v.kind!=='reference'||document.getElementById('reference').checked)&&
  (v.kind!=='candidate'||document.getElementById('candidates').checked)
 ).sort((a,b)=>rank(a[1])-rank(b[1])),points=[];
 for(const [,v] of entries)if(v.fresh)points.push(...v.points);
 if(autoFit&&points.length){
  const xs=points.map(p=>p[0]),ys=points.map(p=>p[1]);
  center=[(Math.min(...xs)+Math.max(...xs))/2,(Math.min(...ys)+Math.max(...ys))/2];
  span=Math.max(4,Math.max(...xs)-Math.min(...xs)+2,Math.max(...ys)-Math.min(...ys)+2);
 }
 const scale=Math.min(w,h)/span;
 const xy=p=>[w/2+(p[0]-center[0])*scale,h/2-(p[1]-center[1])*scale];
 ctx.fillStyle='#101923';ctx.fillRect(0,0,w,h);ctx.font='12px monospace';
 const step=10**Math.floor(Math.log10(span/5));
 ctx.strokeStyle='#25394a';ctx.fillStyle='#8ba4b7';ctx.lineWidth=1;ctx.setLineDash([]);
 for(let x=Math.floor((center[0]-w/scale/2)/step)*step;x<=center[0]+w/scale/2;x+=step){
  const [px]=xy([x,0]);ctx.beginPath();ctx.moveTo(px,0);ctx.lineTo(px,h);ctx.stroke();ctx.fillText(x.toFixed(2),px+3,h-8);
 }
 for(let y=Math.floor((center[1]-h/scale/2)/step)*step;y<=center[1]+h/scale/2;y+=step){
  const [,py]=xy([0,y]);ctx.beginPath();ctx.moveTo(0,py);ctx.lineTo(w,py);ctx.stroke();ctx.fillText(y.toFixed(2),5,py-3);
 }
 for(const [topic,v] of entries){
  if(!v.fresh||!v.points.length)continue;
  const observer=v.observer,prediction=v.kind==='prediction';
  const isPlan=v.kind==='plan',isReference=v.kind==='reference';
  const isCandidate=v.kind==='candidate';
  const isPath=prediction||isPlan||isReference||isCandidate;
  const target=v.target;
  ctx.strokeStyle=ctx.fillStyle=colors[target]||'#ffffff';ctx.lineWidth=isPlan?5:1.5;
  ctx.globalAlpha=isPlan||!isPath?1:.45;
  if(isPath){
   ctx.setLineDash(prediction?[7,5]:isReference?[2,6]:[]);ctx.beginPath();
   v.points.forEach((p,i)=>{const [x,y]=xy(p);i?ctx.lineTo(x,y):ctx.moveTo(x,y)});ctx.stroke();ctx.setLineDash([]);
   if(!isReference)for(const p of v.points){const [x,y]=xy(p);ctx.beginPath();ctx.arc(x,y,isPlan?3:2,0,2*Math.PI);ctx.fill()}
   const [x,y]=xy(v.points[v.points.length-1]);ctx.beginPath();ctx.moveTo(x-5,y-5);ctx.lineTo(x+5,y+5);ctx.moveTo(x-5,y+5);ctx.lineTo(x+5,y-5);ctx.stroke();
   const label=isPlan?`${shortName(observer)} MPC +${v.horizon.toFixed(1)}초`:
    isReference?`${shortName(observer)} 참조 경로`:`${shortName(observer)}가 예측한 ${shortName(target)} (CV)`;
   if(!isCandidate)ctx.fillText(label,x+10,y-12);
  }else{
   const [x,y]=xy(v.points[0]);ctx.beginPath();ctx.arc(x,y,8,0,2*Math.PI);ctx.fill();ctx.fillText(`${shortName(target)} 실제 위치`,x+12,y+18);
  }
 }
 ctx.globalAlpha=1;
 const lines=robots.map(r=>{
  const plan=data.topics[`/${r}/mpc_path`],status=data.planners?.[r];
  const reason=status?.fresh?(descriptions[status.status]||status.status):'Planner 상태 수신 대기/만료';
  const output=plan?.fresh&&plan.points.length?`${plan.points.length}점 · 현재부터 ${plan.horizon.toFixed(1)}초 계획`:'표시할 유효 MPC 경로 없음';
  const control=data.controllers?.[r];
  const driving=control?.fresh?(controlDescriptions[control.status]||control.status):'제어 상태 수신 대기/만료';
  const recovery=data.recoveries?.[r];
  const search=recovery?.fresh?` | 재탐색: ${recoveryDescriptions[recovery.phase]||recovery.phase} · 회차 ${recovery.episode} · 후보 ${recovery.selected}`:'';
  return `${shortName(r)}: ${reason} | ${output} | 제어: ${driving}${search}`;
 });
 document.getElementById('status').textContent=(connected?'':'웹 연결 끊김\n')+(lines.join('\n')||'ROS 토픽 대기 중…');
}
async function poll(){
 try{const r=await fetch('/api/state',{cache:'no-store'});if(!r.ok)throw Error(r.status);data=await r.json();connected=true}
 catch(e){connected=false;data={topics:{},planners:{}}}
 draw();setTimeout(poll,200);
}
window.addEventListener('resize',draw);poll();
</script></html>'''


class TopicStore:
    def __init__(self, stale_after=1.0, robots=DEFAULT_ROBOTS, prediction_source='mpc'):
        self.stale_after = stale_after
        self.robots = tuple(robots)
        self.prediction_source = prediction_source
        self.lock = threading.Lock()
        self.topics = {}
        self.planners = {}
        self.controllers = {}
        self.recoveries = {}

    def update_status(self, robot, status):
        with self.lock:
            self.planners[robot] = dict(status=status, received=time.monotonic())

    def update_control(self, robot, status):
        with self.lock:
            self.controllers[robot] = dict(status=status, received=time.monotonic())

    def update_recovery(self, robot, status):
        try:
            value = json.loads(status)
            if not isinstance(value, dict) or not isinstance(value.get('phase'), str):
                return
            if not all(isinstance(value.get(k), int) for k in ('episode', 'selected')):
                return
            clean = {k: value[k] for k in ('phase', 'episode', 'selected')}
        except (ValueError, TypeError):
            return
        with self.lock:
            self.recoveries[robot] = dict(clean, received=time.monotonic())

    def update(self, topic, frame, points, horizon=0.0):
        identity = topic_identity(topic, self.robots)
        valid = frame == 'world' and all(math.isfinite(v) for p in points for v in p)
        with self.lock:
            self.topics[topic] = dict(
                frame=frame, points=points if valid else [], horizon=horizon,
                received=time.monotonic(), valid=valid,
                **identity,
            )

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            result = {}
            for topic, value in self.topics.items():
                fresh = value['valid'] and now - value['received'] <= self.stale_after
                result[topic] = dict(
                    frame=value['frame'], points=value['points'] if fresh else [],
                    horizon=value['horizon'], fresh=fresh,
                    kind=value['kind'], observer=value['observer'], target=value['target'],
                    reason='입력 만료' if value['valid'] else 'world 좌표계/유한값 확인 필요',
                )
            planners = {robot: dict(status=value['status'],
                                   fresh=now-value['received'] <= self.stale_after)
                        for robot, value in self.planners.items()}
            controllers = {robot: dict(status=value['status'],
                                      fresh=now-value['received'] <= self.stale_after)
                           for robot, value in self.controllers.items()}
            recoveries = {r:dict(phase=v['phase'], episode=v['episode'], selected=v['selected'],
                                  fresh=now-v['received'] <= self.stale_after)
                          for r,v in self.recoveries.items()}
            return {'robots': self.robots, 'prediction_source': self.prediction_source,
                    'topics': result, 'planners': planners, 'controllers': controllers, 'recoveries': recoveries}


def make_handler(store):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = urlsplit(self.path).path
            if path == '/':
                body, content_type = PAGE.encode(), 'text/html; charset=utf-8'
            elif path == '/api/state':
                body = json.dumps(store.snapshot(), allow_nan=False).encode()
                content_type = 'application/json; charset=utf-8'
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', content_type)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *_args):
            pass

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8091)
    parser.add_argument('--robot-count', type=int, choices=(2, 3), default=len(DEFAULT_ROBOTS),
                        help='two defaults to 2; three defaults to 3')
    parser.add_argument('--prediction-source', choices=['mpc', 'diagnostic'], default='mpc',
                        help='Optional CV overlay source; default uses the original C++ MPC input')
    args, ros_args = parser.parse_known_args()
    import rclpy
    from nav_msgs.msg import Odometry, Path
    from std_msgs.msg import String
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init(args=ros_args)
    node = Node('peer_prediction_viewer')
    robots = ('robot_a', 'robot_b', 'robot_c')[:args.robot_count]
    store = TopicStore(robots=robots, prediction_source=args.prediction_source)

    def odom_callback(msg, topic):
        p = msg.pose.pose.position
        store.update(topic, msg.header.frame_id, [[p.x, p.y]])

    def path_callback(msg, topic):
        frames_match = all(p.header.frame_id in ('', msg.header.frame_id) for p in msg.poses)
        horizon = 0.0
        if msg.poses:
            first, last = msg.header.stamp, msg.poses[-1].header.stamp
            horizon = (last.sec - first.sec) + (last.nanosec - first.nanosec) * 1e-9
        store.update(topic, msg.header.frame_id if frames_match else 'mixed_frames',
                     [[p.pose.position.x, p.pose.position.y] for p in msg.poses], horizon)

    server = None
    try:
        for robot in robots:
            predictions = [(f'mpc_peer_prediction/{peer}' if args.prediction_source == 'mpc'
                            else f'peers/{peer}/prediction', Path, path_callback)
                           for peer in robots if peer != robot]
            # Accept the original two-robot layout as well as per-peer topics.
            if robot in ('robot_a', 'robot_b'):
                predictions.append(('mpc_peer_prediction' if args.prediction_source == 'mpc'
                                    else 'peer_prediction', Path, path_callback))
            for suffix, msg_type, callback in (
                ('odom', Odometry, odom_callback), *predictions,
                ('mpc_path', Path, path_callback), ('mpc_reference_path', Path, path_callback),
                *[(f'mpc_candidate_{i}', Path, path_callback) for i in range(5)],
            ):
                topic = f'/{robot}/{suffix}'
                node.create_subscription(msg_type, topic,
                                         lambda msg, t=topic, cb=callback: cb(msg, t),
                                         qos_profile_sensor_data)
            node.create_subscription(String, f'/{robot}/mpc_status',
                                     lambda msg, r=robot: store.update_status(r, msg.data), 1)
            node.create_subscription(String, f'/{robot}/control_status',
                                     lambda msg, r=robot: store.update_control(r, msg.data), 1)
            node.create_subscription(String, f'/{robot}/recovery_status',
                                     lambda msg, r=robot: store.update_recovery(r, msg.data), 1)
        server = ThreadingHTTPServer((args.host, args.port), make_handler(store))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        node.get_logger().info(f'MPC path viewer: http://{args.host}:{args.port} (Ctrl+C to stop)')
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if server is not None:
            server.shutdown()
            server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
