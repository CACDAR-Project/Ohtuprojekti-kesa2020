import argparse

from application.conf.configuration import Configuration
#from application.tools.helpers import str_convert
from application import app


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

    args = parser.parse_args()
    # TODO: iterate args and insert them into conf

    return args


def main():
    conf = Configuration(load_settings_from_file=True)
    args = handle_args(parser=argparse.ArgumentParser(), conf=conf)

    # Here we can choose different app/main/node based on settings/arguments
    app.run()


if __name__ == '__main__':
    main()
