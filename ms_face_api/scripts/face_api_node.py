import cognitive_face as CF
import rospy
from cv_bridge import CvBridge
from cv2 import startWindowThread, imencode, imread
from StringIO import StringIO
from sensor_msgs.msg import Image
from ms_face_api.srv import PersonGroupCreate, Detect
from ms_face_api.msg import Face, Faces
from json import dumps
from math import pi


class CognitiveFaceROS:

    def __init__(self):
        self._api_key = rospy.get_param('~key',
                                        'be062e88698e4777ac6196623d7230dd')
        self._topic_timeout = rospy.get_param('~timeout', 10)
        self._cv_bridge = CvBridge()
        startWindowThread()
        CF.Key.set(self._api_key)
        self._srv_person_group_create = rospy.Service('~person_group_create',
                                                      PersonGroupCreate,
                                                      self._person_group_create
                                                      )
        self._srv_detect = rospy.Service('~detect',
                                         Detect,
                                         self._detect_srv
                                         )

    def _convert_ros2jpg(self, image_msg):
        img = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
        retval, buf = imencode('.jpg', img)
        return buf.tostring()

    def _person_group_create(self, req):
        print req
        return []

    def _detect_srv(self, req):
        if req.filename is not '':
            img = imread(req.filename)
            msg = self._cv_bridge.cv2_to_imgmsg(img, 'bgr8')
        elif req.topic is not '':
            try:
                msg = rospy.wait_for_message(req.topic, Image,
                                             timeout=self._topic_timeout)
            except rospy.ROSException as e:
                rospy.logwarn('could not receive image from topic %s'
                              ' within %f seconds: %s' %
                              (req.topic, self._topic_timeout, e.message))
                return Faces()
        else:
            msg = req.image

        return self._detect(msg)

    def _detect(self, image):
        faces = Faces()
        try:
            data = CF.face.detect(
                StringIO(self._convert_ros2jpg(image)),
                landmarks=True,
                attributes='age,gender,headPose,smile,glasses')
            print data
            for f in data:
                face = Face()
                face.faceId = f['faceId']
                face.person = ''
                face.faceRectangle.x_offset = f['faceRectangle']['left']
                face.faceRectangle.y_offset = f['faceRectangle']['top']
                face.faceRectangle.width = f['faceRectangle']['width']
                face.faceRectangle.height = f['faceRectangle']['height']
                face.faceRectangle.do_rectify = False
                face.faceLandmarks = dumps(f['faceLandmarks'])
                face.gender = f['faceAttributes']['gender']
                face.age = f['faceAttributes']['age']
                face.smile = f['faceAttributes']['smile']
                face.rpy.x = (f['faceAttributes']['headPose']['roll'] /
                              180.0 * pi)
                face.rpy.y = (f['faceAttributes']['headPose']['pitch'] /
                              180.0 * pi)
                face.rpy.z = (f['faceAttributes']['headPose']['yaw'] /
                              180.0 * pi)
                faces.faces.append(face)
            faces.header = image.header
        except Exception as e:
            rospy.logwarn('failed to detect via the MS Face API: %s' %
                          e.message)
        return faces


rospy.init_node('cogntivefaceapi')
cfa = CognitiveFaceROS()
#msg = rospy.wait_for_message('/image', Image)
#print cfa._detect(msg)
rospy.spin()
