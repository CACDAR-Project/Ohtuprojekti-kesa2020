## node_detector_control

### Services

* detector_control_node/add_object_detector
  * Used to add models for detection
    * name (string)
    * model_path (string)
    * label_path (string)
  * detector_control_node/remove_object_detector
    * Used to remove models added with add_object_detector
      * name (string), same that was given with add_object_detector
  * combine_toggle
    * Toggle the combining of observations from all detectors to a single Ros message
      * state (bool)
  * [detector_name]/frequency
    * Changes the rate of image processing of the specified detector to the received frequency in hz
      * frequency (int32)
    * Replies with a new_frequencyResponse (string) after the rate has been changed
  * [detector_name]/toggle
    * Toggles detections with the specified detector
      * state (bool)
      
### Topics

* observations, observations from all active detectors are published here
    * camera_id (string)
    * image_counter (uint64), identifier for the image
    * observations (observation array)
      * observation_type (string), name of the detector used, "QR" for QR
      * class_id (uint16) class_id given by Tensorflow, if observation is not of type QR
      * label (string) mapped to label file with class_id, OR data of a QR code
      * score (float64) confidence score between 0.0-1.0 from Tensorflow 
      * bbox (bbox), bounds of rectangle around the detected object/QR code. Values between 0.0-1.0, offset from top left corner
        * top (float64)
        * right (float64)
        * bottom (float64)
        * left (float64)
      * pol (polygon), more accurate bound for QR observations
        * length (uint8), amount of points
        * points (point64 array), values between 0.0-1.0, offset from top left corner
          * y (float64)
          * x (float64)
* warning, sent when detection takes longer than desired rate
  * message (string)
  
### Ros-parameters

Different models can be set up on startup via a dictionary parameter entry, for an example:

\<param name="testi" type="yaml" value="object_detect: {model_path: 'ssd_mobilenet_v1_1_metadata_1.tflite', label_path: 'mscoco_complete_labels', detect_on: True, frequency: 42}" /\>

Which results in an object-detection node with the name "object_detect".

Only model_path and label_path are required, frequency and detect_on have default values of 5 and True.


## node_camera

### Topics

* camera/images, continuously published frames from the assigned video source as Ros image messages
  * camera_id (string) 
  * image_counter (uint64), identifier for the frames
  * data (uint8 array), the three color channels as a one dimensional byte array
  * height (uint16), height of the image in pixels
  * width (uint16), width of the image in pxels
  
### Parameters

* video_source (string), path to the device or file to publish video from




  
