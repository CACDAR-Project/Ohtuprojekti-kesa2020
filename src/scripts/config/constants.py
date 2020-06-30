## @package config
# This modules stores various variables to be imported into other modules.

# All paths are relative to the ./src directory of the repository.
resources_path = '../resources'
tflite_path = f'{resources_path}/tflite_models'
videos_path = f'{resources_path}/videos'
images_path = f'{resources_path}/images'

# ROS Package/group name
name_ros_pkg = 'konenako'

# Node names
name_node_camera = 'camera'
name_node_detector_control = 'detector_control_node'
name_node_object_detector = 'object_detector'
name_node_qr_detector = 'qr_detector'
name_node_printer = 'printer'
name_node_input = 'input'

# Detector names
name_det_qr = 'QR'

# Rosparameters
# Sleep time between polling for parameter, in seconds
rosparam_poll_interval = 1

# Parameters
rosparam_initial_detectors = 'init_detectors'
rosparam_camera_hz = 'camhz'
rosparam_video_source = 'video_source'
rosparam_video_feed_name = 'video_feed_name'

rosparam_combine_results = 'combine_results'
rosparam_combine_toggle = 'combine_toggle'

rosparam_detection = 'detect_on'

rosparam_model_file = 'model_file'
rosparam_label_file = 'label_file'
rosparam_frequency = 'frequency'

# Topics
topic_images = 'images'
topic_observations = 'observations'
topic_warnings = 'warnings'

# Services
srv_frequency = rosparam_frequency
srv_combine_toggle = rosparam_combine_toggle
srv_toggle = 'toggle'
srv_add_object_detector = 'add_object_detector'
srv_rm_object_detector = 'remove_object_detector'
srv_score_treshold = 'score_treshold'
srv_labels = 'labels'
srv_sort_by = 'sort_by'
srv_filter_by = 'filter_by'