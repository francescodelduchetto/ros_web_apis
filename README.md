# ros_web_apis
Some WEB APIs for ROS usage

## ms_face_api

This is a ROS wrapper for the [Microsoft Cognitive Services FACE API](https://www.microsoft.com/cognitive-services/en-us/face-api). It offers ROS services to detect faces, extract face markers, and recognise attributes like "gender", "smile", and "age". Also, faces can be trained for identification and if a face is trained, the detection will also contain the identity of a person in the image.

### Services:

#### Detection and optional Identification

* Service `cognitivefaceapi/detect`
* Service Definition: [`Detect`](/LCAS/ros_web_apis/blob/master/ms_face_api/srv/Detect.srv)
  * An image can either be defined by defining in the requests 
    1. a `sensor_msgs/Image`,
    2. a `filename`, or
    3. a `topic` where the image is read from (waiting for a timeout)
  * It returns an [`ms_face_api/Faces`](https://github.com/LCAS/ros_web_apis/blob/master/ms_face_api/msg/Faces.msg) data structure
  * if the `identify` flag is `true` in the request, the API tries to identify all the detected faces
