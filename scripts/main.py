from src.apps.task import DefaultName
from src.config.config_funcs import get_configs
from src.apps.car_follow import BasicFollowApp

config = get_configs("irl.local.yml")

DefaultName(config[0], config[1])
#BasicFollowApp(config[0], config[1])
