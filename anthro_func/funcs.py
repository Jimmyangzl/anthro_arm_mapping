import os
import yaml
import argparse
from ament_index_python.packages import get_package_share_directory


def get_parser():
    config_path = os.path.join(
        get_package_share_directory('anthro_arm_mapping'),
        'config',
        'config.yaml'
    )
    with open(config_path, 'r') as file:
        configs = yaml.safe_load(file)
    arg_names = list(configs.keys())
    parser = argparse.ArgumentParser(
        prog='tracker_pub',
        description='Publishes joint position of anthropomorphic pose.')
    for arg_name in arg_names:
        parser.add_argument("--"+arg_name, help="(Option) Specify "+arg_name)
    # args = parser.parse_args()
    return parser