from src.apps.task import DefaultName
from src.config.config_funcs import get_configs

config = get_configs("irl.local.yml")
config2 = get_configs("irl2.local.yml")

DefaultName(config[0], config[1])
DefaultName(config2[0], config2[1])
#BasicFollowApp(config[0], config[1])
