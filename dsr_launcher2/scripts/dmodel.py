#!/usr/bin/env python

from ros import rosparam
import yaml
import tempfile
from resource_retriever import get
import subprocess
import signal
import argparse
import os
import sys

def show_description():
    return "roslaunch all of the nodes"


def show_usage(cmd=None):
    cmd = os.path.relpath(sys.argv[0], os.getcwd()) if cmd is None else cmd
    return "{0} [-h|--help] [--version]".format(cmd)


def show_epilog():
    return "epilog..."

class create_robot_model():
    def __init__(self, base_file=None):
        if base_file:
            self.data = yaml.load(base_file)
        else:
            self.data = {}
    def write(self, f):
        f.write(yaml.dump(self.data, default_flow_style=False))

    def add_display(self, name, class_name, topic=None, color=None, fields={}, enabled=True):
        d = {'Name': name, 'Class': class_name, 'Enabled': enabled}
        if topic:
            d['Topic'] = topic
        if color:
            d['Color'] = '%d; %d; %d' % color
        d.update(fields)
        self.data['Visualization Manager']['Displays'].append(d)

    def add_model(self, parameter='robot_description', tf_prefix=None):
        fields = {'Robot Description': parameter}
        if tf_prefix:
            fields['TF Prefix'] = tf_prefix
        self.add_display(tf_prefix[1:], 'rviz/RobotModel', fields=fields)


if __name__=="__main__":
    # Ref : https://docs.python.org/2/library/argparse
    parser = argparse.ArgumentParser(description=show_description(),
                                     usage=show_usage(),
                                     epilog=show_epilog(),
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('-o', '--option', type=str, default='eof', help='dont care')
    parsed_known_args, unknown_args = parser.parse_known_args(sys.argv[1:])
    print(parsed_known_args)
    usp = None
    try:
        l = rosparam.list_params('')
        models = []
        for p in l:
            v = rosparam.get_param(p)
            if type(v) is str and len(v) > 10000:
                models.append(p)
        r = create_robot_model(get('package://dsr_launcher/rviz/default.rviz'))
        for m in models:
            r.add_model(m, m[0:-18])
        temp = tempfile.NamedTemporaryFile()
        r.write(temp)
        temp.flush()
        args = ['rosrun', 'rviz', 'rviz', '-d', temp.name]
        usp = subprocess.call(args)
        temp.close()
    except KeyboardInterrupt:
        print("Interrupted by user, shutting down...")
