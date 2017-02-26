import cognitive_face as CF
import rospy
from cv_bridge import CvBridge
from cv2 import startWindowThread, imencode
from StringIO import StringIO
from sensor_msgs.msg import Image
from ms_face_api.srv import PersonGroupCreate


class CognitiveFaceROS:

    def __init__(self):
        self._api_key = rospy.get_param('~key',
                                        'be062e88698e4777ac6196623d7230dd')
        self._cv_bridge = CvBridge()
        startWindowThread()
        CF.Key.set(self._api_key)
        self._srv_person_group_create = rospy.Service('person_group_create',
                                                      PersonGroupCreate,
                                                      self.person_group_create)

    def person_group_create(self, req):
        print req
        return []

    def detect(self, image):
        img = self._cv_bridge.imgmsg_to_cv2(image, "bgr8")
        retval, buf = imencode('.jpg', img)
        strio = StringIO(buf.tostring())
        return CF.face.detect(strio,
                              landmarks=True,
                              attributes='age,gender,headPose,smile,glasses')


rospy.init_node('cogntivefaceapi')
cfa = CognitiveFaceROS()
#msg = rospy.wait_for_message('/image', Image)
#print cfa.detect(msg)
rospy.spin()
