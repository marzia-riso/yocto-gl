#! /usr/bin/env python3 -B

import click
import sys
import glob
import os
import subprocess
import json
import random


@click.group()
def cli():
    pass


@cli.command()
@click.argument('meshes_dir')
@click.argument('svgs_dir')
@click.argument('output_jsons_dir')
def gui(meshes_dir, svgs_dir, output_jsons_dir):
    mesh_names = glob.glob(f'{meshes_dir}/meshes/*.ply')
    mesh_num = len(mesh_names)

    svgs_names = glob.glob(f'{svgs_dir}/*.svg')
    svgs_num = len(svgs_names)

    try:
        os.mkdir(output_jsons_dir)
    except:
        pass

    output_jsons_names = glob.glob(f'{output_jsons_dir}/*.json')
    output_jsons_names = [os.path.basename(filename).split(
        '.')[0] for filename in output_jsons_names]

    print(output_jsons_names)
    for mesh_id, mesh_name in enumerate(mesh_names):
        name = os.path.basename(mesh_name).split('.')[0]
        if name in output_jsons_names:
            continue

        msg = f'[{mesh_id+1}/{mesh_num}] {mesh_name}'
        print(msg + ' ' * max(0, 78-len(msg)))

        first = random.randint(0, svgs_num)
        second = random.randint(0, svgs_num)

        while (first == second):
            second = random.randint(0, svgs_num)

        first_svg = svgs_names[first]
        second_svg = svgs_names[second]

        cmd = f'.\\bin\\gui.exe --svg {first_svg} --output-test {output_jsons_dir}/{name}.json {mesh_name}'
        print(cmd)

        subprocess.run(cmd, shell=True).returncode


@cli.command()
@click.argument('jsons_dir')
@click.argument('output_jsons_dir')
def operation(jsons_dir, output_jsons_dir):
    json_names = glob.glob(f'{jsons_dir}/tests/*.json')
    json_num = len(json_names)
    operations = ['op_union', 'op_difference',
                  'op_intersection', 'op_symmetrical_difference']

    for json_id, json_name in enumerate(json_names):
        name = os.path.basename(json_name).split('.')[0]
        dir = os.path.dirname(json_name)

        msg = f'[{json_id+1}/{json_num}] {json_name}'
        print(msg + ' ' * max(0, 78-len(msg)))

        with open(json_name) as json_handler:
            js = json.load(json_handler)
            polygons = js['polygons']
            num_polygons = len(polygons)

            if num_polygons < 3:
                continue

            a = random.randint(1, num_polygons-1)
            b = random.randint(1, num_polygons-1)

            while (a == b or polygons[a] == [] or polygons[b] == []):
                b = random.randint(0, num_polygons-1)

            type = random.randint(0, len(operations))
            print(f'{num_polygons} {a} {b} - {type}\n')

            operation = {'a': a, 'b': b, 'type': type}
            js['operations'].append(operation)

            with open(f'{output_jsons_dir}/tests/{name}.json', 'w') as json_file:
                json.dump(js, json_file, indent=2)


@cli.command()
@click.argument('tests_dir')
@click.argument('output_tests_dir')
def fix(tests_dir, output_tests_dir):
    test_names = glob.glob(f'{meshes_dir}/tests/*.json')
    test_num = len(test_names)

    try:
        os.mkdir(output_tests_dir)
    except:
        pass

    for test_id, test_name in enumerate(test_names):
        name = os.path.basename(test_name).split('.')[0]

        msg = f'[{test_id+1}/{test_num}] {test_name}'
        print(msg + ' ' * max(0, 78-len(msg)))

        cmd = f'.\\bin\\gui.exe --output-test {output_tests_dir}/{name}.json {test_name}'
        print(cmd)

        subprocess.run(cmd, shell=True).returncode


cli()
