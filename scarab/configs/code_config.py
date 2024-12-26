import os


DEBUG = False # in DEBUG mode command to servos is not issued

project_dir = os.path.join(os.path.dirname(__file__), '..')
main_log_file = os.path.join(project_dir, 'logs', 'main.log')

movement_command_file = os.path.join(project_dir, 'wrk', 'movement_command.txt')
neopixel_command_file = os.path.join(project_dir, 'wrk', 'neopixel_command.txt')

cache_dir = os.path.join(project_dir, 'cache')

logger_config = {
    'version': 1,
    'formatters': {
        'default_formatter': {
            'format': '[%(asctime)s][%(levelname)s] %(message)s'
        },
    },
    'handlers': {
        'main_file_handler': {
            'class': 'logging.FileHandler',
            'formatter': 'default_formatter',
            'filename': main_log_file
        },
    },
    'loggers': {
        'main_logger': {
            'handlers': ['main_file_handler'],
            'level': 'DEBUG',
            'propagate': True
        },
    }
}
