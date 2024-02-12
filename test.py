import json

path = "/home/robert/catkin_ws/src/bag_crawler/.bag_crawler_global_config.json"
config = {"elements_of_control": {"/joy_local/cmd_vel": "robot_gamepad", "/nav/cmd_vel": "autonomous"},
          "panorama topics": None}
with open(path, "w", encoding="utf-8") as file:
    json.dump(config, file, indent=4)


