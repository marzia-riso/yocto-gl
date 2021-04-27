#! /usr/bin/env python3 -B

import click
import sys
import glob
import os
import subprocess
import json


@click.command()
@click.argument('bin', type=click.Path(exists=True))
@click.argument('dirname', type=click.Path(exists=True))
@click.argument('task')
def test(bin, dirname, task):
    if task == 'convert':
        input_paths = glob.glob(f'{dirname}/meshes/*.ply')
        output = f'{dirname}/tests'
        drawing = 'data/svgs/abc.json'
    elif task == 'test':
        input_paths = glob.glob(f'{dirname}/tests/*.json')
        output = f'{dirname}/images'
    
    input_num = len(input_paths)

    try:
        os.mkdir(output)
    except:
        pass

    result = {}
    result['num_tests'] = 0
    result['num_errors'] = 0
    result['errors'] = []
    result['num_ok'] = 0
    result['ok'] = []
    result['num_OS_errors'] = 0
    result['OS_errors'] = []
    result['num_timeouts'] = 0
    result['timeouts'] = []

    append = ''
    for input_id, input_path in enumerate(input_paths):
        result['num_tests'] += 1
        name = os.path.basename(input_path).split('.')[0]
        msg = f'[{input_id}/{input_num}] {input_path}'
        print(msg + ' ' * max(0, 78-len(msg)))

        if task == 'convert':
            cmd = f'{bin} --model {input_path} {drawing} --output-json {output}/{name}.json'
        elif task == 'test':
            cmd = f'{bin} {input_path} --output {output}/{name}.png'
        print(cmd)
        
        try:
            retcode = subprocess.run(cmd, timeout=60, shell=True).returncode
            if retcode == 0:
                result['num_ok'] += 1
                result['ok'] += [input_path]
            elif retcode == -8:
                result['num_high_genus'] += 1 
                result['high_genus'] += [input_path] 
            else:
                result['num_errors'] += 1
                result['errors'] += [input_path]

        except OSError:
            result['num_OS_errors'] += 1
            result['OS_errors'] += [input_path]

        except subprocess.TimeoutExpired:
            result['num_timeouts'] += 1
            result['timeouts'] += [input_path]

        except:
            result['num_errors'] += 1
            result['errors'] += [input_path]

        with open(f'{output}/trace-result.json', 'wt') as f:
            json.dump(result, f, indent=2)


if __name__ == '__main__':
    test()
