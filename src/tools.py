from sys import platform
import config

def port_return():
    if platform.startswith('linux'):
        return config.PORT_NAME_LINUX
    elif platform.startswith('win') or platform.startswith('cygwin'):
        return config.PORT_NAME_WINDOWS
    elif platform.startswith('darwin'):
        return config.PORT_NAME_OSX

def find_methods(obj):
        return [method for method in dir(obj) if callable(getattr(obj, method)) and not method.startswith('_')]

def make_config():
    config_list = []
    for var in dir(config):
        if not var.startswith('_'):
            obj = {'name': var, 'value': eval("config.{}".format(var))}
            config_list.append(obj)
    return config_list

def change_config(data):
    for var in data:
        x = var["value"]
        if x == "true":
            x = "True"
        if x == "false":
            x = "False"
        elif not x.isdigit():
            exec 'config.{} = "{}"'.format(var["name"], x)
        exec 'config.{} = {}'.format(var["name"], x)
    print config.BATTERY_CELL_COUNT