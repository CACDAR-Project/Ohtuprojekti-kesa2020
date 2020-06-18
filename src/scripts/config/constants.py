## @package config
# This modules stores various variables to be imported into other modules.

# All paths are relative to the ./src folder of the repository.
resources_path = '../resources'
tflite_path = f'{resources_path}/tflite_models'
videos_path = f'{resources_path}/videos'
images_path = f'{resources_path}/images'

# # Work in progress, these will change when the other PR is merged
# Node names
name_node_camera = 'camera'  #
name_node_detector_control = 'detector_control_node'  #
name_node_object_detector = 'object_detector'
name_node_qr_detector = 'qr_detector'
name_node_printer = 'printer'  #

# Parameters
rosparam_video_source = 'video_source'  #
rosparam_video_feed_name = 'video_feed_name'  #
rosparam_combine_results = 'combine_results'  #
rosparam_detection = 'detect_on'
rosparam_model_file = 'model_file'
rosparam_label_file = 'label_file'
rosparam_frequency = 'frequency'

# Topics
topic_images = 'images'  #
topic_observations = 'observations'
topic_warnings = 'warnings'

# Services
srv_frequency = 'frequency'
srv_toggle = 'toggle'
srv_add_object_detector = 'add_object_detector'
srv_rm_object_detector = 'remove_object_detector'
