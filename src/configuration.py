'''Configuration module for the application.

Consains variables (dictionaries) for setting default variables. Also provides the Settings-class that is used
for loading and managing the settings. Only one instance of this class should be created.
'''

from typing import Union, List, Tuple, Dict, Optional
from tools import filereaderwriter as fileio
from tools.helpers import str_convert

# Default settings for the application can be set here
# Directories
paths = {'SETTINGS': 'config', 'IMAGES': 'images', 'DATA': 'data'}

# Filenames
filenames = {'SETTINGS': 'settings'}

# Various settings
settings = {
    'settings_comment_sign': '#',
    'settings_assignment_sign': '=',
    'CAMERA_ID': 10
}


class Initializer():
    '''Class that holds static methods to help with initializing'''
    def initialize_directories(paths: Dict[str, str]) -> None:
        print('Configured directories:')
        for key, directory in paths.items():
            if not fileio.check_directory_exists(directory):
                print(
                    f'./{directory}/ directory for {key} not found, attempting to create..'
                )
                # create_directory will raise an exception if creation is unsuccessful, and app will exit
                fileio.create_directory(directory)

            print(f'Using ./{directory}/ for {key}!')
        print('All directories initialized.')
        print()

    def filter_map_settings(
            settings_list: List[str],
            comment_sign: str = '#',
            assignment_sign: str = '=') -> List[Tuple[str, str]]:
        '''This function takes one list containing lines read from a settings file. It filters
        out the comments and faulty lines and returns a list containing tuples where each tuple
        represents a variable and value pair.
        
        File should be of the following format:
        variable1=value
        variablen=value
        That is one variable with the specified setting per line. Empty and comment lines are ignored.
        Returns a list of tuples, each one containing a variable and value pair.
        If the file is empty or could not be read returns an empty List.
        '''
        def valid_line_filter(line: str) -> bool:
            '''Returns False for lines that are empty or starts with commentsign and
            all lines that does not have exactly one assignment sign('=')
            '''
            if not line: return False
            return (not line.startswith(comment_sign)
                    ) and line.count(assignment_sign) == 1

        settings = filter(valid_line_filter, settings_list)
        return list(
            tuple(map(str.strip, l.split(assignment_sign))) for l in settings)


class Configuration():
    '''Class that holds all settings. By default loads settings from disk.
    Only one instance should be created of this class and that object is assigned
    to the class variable Configuration._singleton_obj .'''

    _singleton_obj = None

    def __init__(self, load_from_file: bool = True) -> None:
        '''Constructor. After the object has been created the object will be assigned to the class variable _singleton_obj.
        '''
        if Configuration._singleton_obj:
            raise RuntimeError(
                'Settings object already created, cannot instantiate a new one. Please use the existing one.'
            )
            #del self
            #return

        self.paths = paths.copy()
        self.filenames = filenames.copy()
        self.settings = settings.copy()

        Initializer.initialize_directories(self.paths)

        if load_from_file:
            self.load_settings_from_file()

        Configuration._singleton_obj = self

    def load_settings_from_file(self) -> None:
        '''Reads the settings, converts them into the right type and loads them'''
        settings_file = fileio.build_path(self.paths['SETTINGS'],
                                          self.filenames['SETTINGS'])
        print(f'Using settings file \'./{settings_file}\'.')

        settings_read = Initializer.filter_map_settings(
            fileio.read_file_lines(settings_file),
            self.settings['settings_comment_sign'],
            self.settings['settings_assignment_sign'])

        if len(settings_read) != 0:
            print(f'Using {len(settings_read)} setting(s) from file:')

            # The list contains tuples, but we still need to unpack them with map here
            for var, val in map(lambda pair: (pair[0], str_convert(pair[1])),
                                settings_read):
                print(var, '=', val, 'loaded from disk.')
                self.settings[var] = val
        else:
            print('No settings loaded, using default values.')
        print()

    def __str__(self) -> str:
        def join_helper(pair):
            return '='.join(pair)

        paths_vars = 'Paths:\n' + '\n'.join(
            (map(join_helper, self.paths.items())))
        files_vars = 'Files:\n' + '\n'.join(
            (map(join_helper, self.filenames.items())))
        # Settings dictionary can contain values that are not type(str) -> map to strings
        settings_vars = 'Settings:\n' + '\n'.join((map(
            join_helper, map(lambda t:
                             (t[0], str(t[1])), self.settings.items()))))

        return '\n\n'.join(
            ('Loaded settings:', paths_vars, files_vars, settings_vars))

    def get_instance(self):
        return Configuration.get_instance()

    @staticmethod
    def get_instance(
    ):  #-> Optional[Configuration]: <- fails with error: name 'Configuration' is not defined ??
        '''Static method that returns the only instance of this class.'''
        return Configuration._singleton_obj
