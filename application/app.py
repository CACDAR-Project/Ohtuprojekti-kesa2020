import argparse

from application.conf.configuration import Configuration

def run_facedetection():
    pass

def run_something_else():
    pass

def handle_args(parser, conf):
    parser.add_argument('-m',
                        '--model_file',
                        default=conf.settings['DEFAULT_MODEL'],
                        help='.tflite model to be executed')
    parser.add_argument('-l',
                        '--label_file',
                        default=conf.settings['DEFAULT_LABELS'],
                        help='name of file containing labels')
    parser.add_argument('--input_mean',
                        default=127.5,
                        type=float,
                        help='input_mean')
    parser.add_argument('--input_std',
                        default=127.5,
                        type=float,
                        help='input standard deviation')
    parser.add_argument('-c',
                        '--camera_id',
                        default=conf.settings['VIDEO_ID'],
                        help='number of the a webcamera')

    return parser.parse_args()

def run():
    ''' This function starts the application'''
    conf = Configuration(load_settings_from_file=False)
    args = handle_args(parser=argparse.ArgumentParser(), conf=conf)
    
    
    print('We are up and running with the following args:')
    print(args)
    print('Thats enough for now, exiting..')
    


if __name__ == '__main__':
    run()
