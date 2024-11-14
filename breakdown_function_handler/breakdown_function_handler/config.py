# Define whether toy robots are used
USE_TOY_ROBOTS = False

# Description of robots
DESCRIPTION_ROBOT = {
    "excavator": {'width': 100, 'height': 100, 'functions': ["Excavation", "Unloading"]},
    "dump_truck": {'width': 50, 'height': 100, 'functions': ["Loading", "Unloading"]},
}

# Navigation functions
NAVIGATION_FUNCTIONS = [
    "avoid_areas_for_all_robots",
    "avoid_areas_for_specific_robots",
    "target_areas_for_all_robots",
    "target_areas_for_specific_robots",
    "allow_areas_for_all_robots",
    "allow_areas_for_specific_robots",
    "return_to_start_for_all_robots",
    "return_to_start_for_specific_robots"
]

# Robot names
ROBOT_NAMES = {
    "robot_dump_truck_01": {"id": "robot_dump_truck_01", "type": "dump_truck"},
    "robot_dump_truck_02": {"id": "robot_dump_truck_02", "type": "dump_truck"},
    # "robot_dump_truck_03": {"id": "robot_dump_truck_03", "type": "dump_truck"},
    # "robot_dump_truck_04": {"id": "robot_dump_truck_04", "type": "dump_truck"},
    # "robot_dump_truck_05": {"id": "robot_dump_truck_05", "type": "dump_truck"},
    # "robot_dump_truck_06": {"id": "robot_dump_truck_06", "type": "dump_truck"},
    "robot_excavator_01": {"id": "robot_excavator_01", "type": "excavator"},
    # "robot_excavator_02": {"id": "robot_excavator_02", "type": "excavator"},
}

# Robot status
ROBOT_STATUS = {
    "robot_dump_truck_01": "idle",
    "robot_dump_truck_02": "idle",
    # "robot_dump_truck_03": "idle",
    # "robot_dump_truck_04": "idle",
    # "robot_dump_truck_05": "idle",
    # "robot_dump_truck_06": "idle",
    "robot_excavator_01": "idle",
    # "robot_excavator_02": "idle",
}

