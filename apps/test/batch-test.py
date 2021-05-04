#! /usr/bin/env python3 -B

import click
import sys
import glob
import os
import subprocess
import json


@click.group()
def cli():
    pass


@cli.command()
@click.argument('bin')
@click.argument('dirname')
def svg(bin, dirname):
    mesh_names = glob.glob(f'{dirname}/meshes/*.ply')
    mesh_num = len(mesh_names)

    output = f'{dirname}/output'
    images_dir = f'{output}/images'
    try:
        os.mkdir(output)
        os.mkdir(images_dir)
    except:
        pass

    result = {}
    result['num_tests'] = 0
    result['num_errors'] = 0
    result['errors'] = []
    result['num_ok'] = 0
    result['ok'] = []

    append = ''
    for mesh_id, mesh_name in enumerate(mesh_names):
        result['num_tests'] += 1
        name = os.path.basename(mesh_name).split('.')[0]
        msg = f'[{mesh_id}/{mesh_num}] {mesh_name}'
        print(msg + ' ' * max(0, 78-len(msg)))

        cmd = f'{bin} --model {mesh_name} --output {images_dir}/{name}.png data/svgs/abc.json'
        print(cmd)
        if append == '':
            append = '--append-timings'

        try:
            retcode = subprocess.run(cmd, timeout=60, shell=True).returncode
            if retcode == 0:
                result['num_ok'] += 1
                result['ok'] += [mesh_name]
            else:
                result['num_errors'] += 1
                result['errors'] += [mesh_name]
        except:
            result['num_errors'] += 1
            result['errors'] += [mesh_name]
        
        # except OSError:
        #     result['num_errors'] += 1
        #     result['errors'] += [mesh_name]

        # except subprocess.TimeoutExpired:
        #     result['num_errors'] += 1
        #     result['errors'] += [mesh_name]



    with open(f'{output}/trace-result.json', 'wt') as f:
        json.dump(result, f, indent=2)


@cli.command()
@click.argument('bin')
@click.argument('dirname')
def jsons(bin, dirname):
    jsons_names = glob.glob(f'{dirname}/tests/*.json')
    jsons_num = len(jsons_names)

    output = f'{dirname}/output'
    images_dir = f'{output}/images'
    try:
        os.mkdir(output)
        os.mkdir(images_dir)
    except:
        pass

    result = {}
    result['num_tests'] = 0
    result['num_errors'] = 0
    result['errors'] = []
    result['num_ok'] = 0
    result['ok'] = []

    append = ''
    for json_id, json_name in enumerate(jsons_names):
        result['num_tests'] += 1
        name = os.path.basename(json_name).split('.')[0]
        msg = f'[{json_id}/{jsons_num}] {json_name}'
        print(msg + ' ' * max(0, 78-len(msg)))

        cmd = f'{bin} {json_name} --output {images_dir}/{name}.png'
        print(cmd)
        if append == '':
            append = '--append-timings'

        try:
            retcode = subprocess.run(cmd, timeout=60, shell=True).returncode
            if retcode == 0:
                result['num_ok'] += 1
                result['ok'] += [json_name]
            else:
                result['num_errors'] += 1
                result['errors'] += [json_name]

        except OSError:
            result['num_errors'] += 1
            result['errors'] += [json_name]

        except subprocess.TimeoutExpired:
            result['num_errors'] += 1
            result['errors'] += [json_name]

    with open(f'{output}/trace-result.json', 'wt') as f:
        json.dump(result, f, indent=2)


cli()
