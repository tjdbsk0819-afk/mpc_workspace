"""Own a fresh simulator and execute a single-MPC benchmark inside that container."""
import argparse
import os
import random
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=list('ABD'), default='A')
    parser.add_argument('--ego', choices=[f'robot_{r}' for r in 'abcd'], default='robot_a')
    parser.add_argument('--timeout', type=float, help=argparse.SUPPRESS)  # Ignored for compatibility.
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    backend = 'dmpc' if (root/'dmpc_runtime').is_dir() else 'mpc'
    project = f'antbot-gazebo-{backend}'
    name = project+'-benchmark-sim'
    reference = root/'gazebo/references'/f'{args.scenario}.json'
    if not reference.exists():
        parser.error(f'Missing reference: {reference}')
    compose = ['docker','compose','--project-name',project,'--project-directory',str(root),'-f',str(root/'gazebo/compose.yaml')]
    # Never overlap with an existing drive/sim on the same ROS domain/partition.
    running = subprocess.check_output(['docker','ps','--format','{{.Names}}'],text=True).splitlines()
    conflicts = [n for n in running if n.startswith(project) and ('sim' in n or n.endswith('-drive'))]
    if conflicts:
        parser.error('Stop the existing simulator/drive first: '+', '.join(conflicts))
    env = dict(os.environ)
    env['ROS_DOMAIN_ID'] = os.environ.get('BENCHMARK_ROS_DOMAIN_ID', str(random.SystemRandom().randint(60, 90)))
    print('Benchmark ROS_DOMAIN_ID='+env['ROS_DOMAIN_ID'],flush=True)
    env.setdefault('LOCAL_UID',str(os.getuid())); env.setdefault('LOCAL_GID',str(os.getgid()))
    sim_script = '/workspace/gazebo/sim.sh' if backend=='dmpc' else '/workspace/gazebo/scripts/sim.sh'
    entrypoint = '/workspace/gazebo/entrypoint.sh' if backend=='dmpc' else '/workspace/gazebo/scripts/entrypoint.sh'
    started = False
    try:
        subprocess.run(compose+['run','--rm','-d','--name',name,'runtime','bash',sim_script,args.scenario,'4'],env=env,check=True)
        started = True
        command=['docker','exec',name,'bash',entrypoint,'bash','-c',
                 'echo $$ > /tmp/single_mpc_measurement.pid; exec "$@"','measurement',
                 'python3','/workspace/gazebo/single_mpc/benchmark.py','--scenario',args.scenario,
                 '--ego',args.ego,'--reference',f'/workspace/gazebo/references/{args.scenario}.json']
        measurement=subprocess.Popen(command,env=env,start_new_session=True)
        try:
            return measurement.wait()
        except KeyboardInterrupt:
            # docker exec does not forward terminal SIGINT to the remote Python process.
            subprocess.run(['docker','exec',name,'bash','-c',
                            'kill -INT "$(cat /tmp/single_mpc_measurement.pid)"'],check=False)
            try:
                return measurement.wait(timeout=45)
            except subprocess.TimeoutExpired:
                return 130
    except KeyboardInterrupt:
        return 130
    finally:
        if started:
            subprocess.run(['docker','stop','--signal','SIGINT','--timeout','30',name],stdout=subprocess.DEVNULL)


if __name__ == '__main__':
    sys.exit(main())
