#!/usr/bin/env python3
import argparse
import subprocess


def list_controllers():
    result = subprocess.run(["ros2", "control", "list_controllers"], stdout=subprocess.PIPE)
    output = result.stdout.decode('utf-8').splitlines()
    return output

def get_controllers():
    controllers = list()
    active_controller = None

    list_of_controllers = list_controllers()
    for line in list_of_controllers:
        controller_name = line.split('[')[0]
        controller_state = line.split()[1]

        controllers.append(controller_name)
        if controller_state == "active" and controller_name.endswith("_controller"):
            active_controller = controller_name
    
    return controllers, active_controller

def switch_controller(active_controller, inactive_controller):
    print(f"Deactivating {active_controller} and activating {inactive_controller}...")
    cmd = [
        "ros2", "control", "switch_controllers",
        "--deactivate", active_controller,
        "--activate", inactive_controller
    ]
    result = subprocess.run(cmd, stdout=subprocess.PIPE)
    print(result.stdout.decode('utf-8'))
    
def main(controller):    
    controllers, active_controller = get_controllers()

    if controller not in controllers:
        print(f"Controller '{controller}' is not loaded.")
    elif controller == active_controller:
        print(f"Controller '{controller}' is already active.")
    else:
        switch_controller(active_controller, controller)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('controller', help='Name of the controller to switch to.')
    args = parser.parse_args()

    main(args.controller)
