from sys import platform
import config

def port_return():
    if platform.startswith('linux'):
        return config.PORT_NAME_LINUX
    elif platform.startswith('win') or platform.startswith('cygwin'):
        return config.PORT_NAME_WINDOWS
    elif platform.startswith('darwin'):
        return config.PORT_NAME_OSX