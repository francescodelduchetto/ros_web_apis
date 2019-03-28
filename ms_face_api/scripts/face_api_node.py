#!/usr/bin/env python
from ms_face_api import Key, BaseUrl, face as CF, person_group as PG
from ms_face_api import person as PERSON
from ms_face_api.util import CognitiveFaceException

import rospy
from cv_bridge import CvBridge
from cv2 import startWindowThread, imencode, imread
from StringIO import StringIO
from sensor_msgs.msg import Image
from ms_face_api.srv import PersonGroupSelect, Detect
from ms_face_api.srv import AddIdentityFace
from ms_face_api.srv import DeletePerson
from ms_face_api.msg import Face, Faces, GroupList
from json import dumps, loads
import math 

# big hack to stop annoying warnings
try:
    from requests.packages.urllib3 import disable_warnings
    disable_warnings()
except:
    pass
#

class CognitiveFaceROS:

    def __init__(self):
        self._api_key = rospy.get_param('~key',
                                        'be062e88698e4777ac6196623d7230dd')
        self._topic_timeout = rospy.get_param('~timeout', 10)
        self._base_url = rospy.get_param('~base_url',
                                         'https://uksouth.api.cognitive.microsoft.com/face/v1.0/')
        self._cv_bridge = CvBridge()
        startWindowThread()
        Key.set(self._api_key)
        BaseUrl.set(self._base_url)
        self._srv_person_group_select = rospy.Service('~person_group_select',
                                                      PersonGroupSelect,
                                                      self._person_group_select
                                                      )
        self._srv_detect = rospy.Service('~detect',
                                         Detect,
                                         self._detect_srv
                                         )

        self._srv_add = rospy.Service('~add_face',
                                         AddIdentityFace,
                                         self._add_face_srv
                                         )
                                         
        self._srv_delete = rospy.Service('~delete_person',
                                         DeletePerson,
                                         self._delete_person_srv
                                         )

        self._person_group_id = rospy.get_param('~person_group',
                                                'default')

        self._autotrainning_mode = rospy.get_param('~autotraining_mode',False)
        self._add_person_minarea = rospy.get_param('~add_person_minarea',2800.0)
        self._add_person_maxroll = rospy.get_param('~add_person_maxroll',30.0)
        self._add_person_maxyawl = rospy.get_param('~add_person_maxyawl',0.8)
    
        self._init_person_group(self._person_group_id,False)

        rospy.loginfo('cognitivefaceapi started')
        
    def _init_person_group(self, gid, delete_first=False):
        person_group = None
        try:
            person_group = PG.get(gid)
            rospy.loginfo('getting person group "%s"'
                          % person_group)
        except CognitiveFaceException as e:
            rospy.loginfo('person group "%s" doesn\'t exist, needs creating.'
                          ' Exception: %s'
                          % (gid, e))
        try:
            # if we are expected to reinitialise this, and the group
            # already existed, delete it first
            if person_group is not None and delete_first:
                rospy.loginfo('delete existing person group "%s"'
                              ' before re-creating it.' % gid)
                PG.delete(gid)
                person_group = None
            # if the group does not exist, we gotto create it
            if person_group is None:
                rospy.loginfo('creating new person group "%s".'
                              % gid)
                PG.create(gid, user_data=dumps({
                    'created_by': rospy.get_name()
                }))
            rospy.loginfo('active person group is now "%s".' % gid)
            self._person_group_id = gid
        except CognitiveFaceException as e:
            rospy.logwarn('Operation failed for person group "%s".'
                          ' Exception: %s'
                          % (gid, e))

    def _find_person_by_name(self, name):
        persons = PERSON.lists(self._person_group_id)
        for p in persons:
            if p['name'] == name:
                return p
        return None

    def _init_person(self, name, delete_first=False):
        gid = self._person_group_id
        person = self._find_person_by_name(name)
        try:
            # if we are expected to reinitialise this, and the group
            # already existed, delete it first
            if person is not None and delete_first:
                rospy.loginfo('delete existing person "%s"'
                              ' before re-creating it.' % name)
                PERSON.delete(gid, person['personId'])
                person = None
            # if the group does not exist, we gotto create it
            if person is None:
                rospy.loginfo('creating new person "%s".'
                              % name)
                PERSON.create(gid, name, user_data=dumps({
                    'created_by': rospy.get_name()
                }))
                person = self._find_person_by_name(name)
        except CognitiveFaceException as e:
            rospy.logwarn('Operation failed for person "%s".'
                          ' Exception: %s'
                          % (gid, e))
        return person

    def _convert_ros2jpg(self, image_msg):
        img = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
        retval, buf = imencode('.jpg', img)
        return buf.tostring()

    def _add_face_srv(self, req):
        img_msg = self._get_image(req)
        faces = self._detect(img_msg, False)
        target_face = None        
        
        
        if req.target.do_rectify is True:
            
            target=loads(req.target)
            min_dist = 1000.0
            
            center_target_x=float(req.target.x_offset+ (req.target.width /2.0))
            center_target_y=float(req.target.y_offset + (req.target.height/2.0))
            
            for f in faces.faces:
                center_x = float(f.faceRectangle.x_offset + (f.faceRectangle.width/2.0))
                center_y = float(f.faceRectangle.y_offset + (f.faceRectangle.height/2.0))
                distance=math.sqrt(math.pow((center_x-center_target_x), 2)+math.pow((center_y-center_target_y), 2))
                if distance < min_dist:
                    min_dist = distance
                    target_face = f
            target_face.person = req.person            
            
            if min_dist > 10: 
                target_face= None
   
        else: # biggest face
            max_area = 0.0
            biggest_face = None
            for f in faces.faces:
                area = float(f.faceRectangle.width * f.faceRectangle.height)
                if area > max_area:
                    max_area = area
                    biggest_face = f
            biggest_face.person = req.person
            
            target_face=biggest_face


        self._add_face_target(img_msg,target_face)

        return target_face
            


    def _person_group_select(self, req):
        self._init_person_group(req.id, req.delete_group)
        persons=PERSON.lists(self._person_group_id)
        resp=GroupList()
        persons_list=[]
        for p in persons:
            persons_list.append(str(p['name']))
            
        resp.list=persons_list
        return resp

    def _get_image(self, req):
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
                raise
        else:
            msg = req.image
        return msg

    def _detect_srv(self, req):
        
        if self._autotrainning_mode:
            return self._identify_autotrainning(self._get_image(req))
        else:
            return self._detect(self._get_image(req), req.identify)

    def _detect(self, image, identify=False):
        faces = Faces()
        try:
            data = CF.detect(
                StringIO(self._convert_ros2jpg(image)),
                landmarks=True,
                attributes='age,gender,headPose,smile,glasses,hair,facialhair,accessories')
            identities = {}
            if identify and len(data) > 0:
                rospy.loginfo('trying to identify persons')
                ids = [f['faceId'] for f in data]
                try:
                    identified = CF.identify(ids[:10], self._person_group_id)
                    for i in identified:
                        if len(i['candidates']) > 0:
                            pid = i['candidates'][0]['personId']
                            person = PERSON.get(self._person_group_id, pid)
                            identities[i['faceId']] = person['name']
                    rospy.loginfo('identified %d persons in this image: %s'
                                  % (len(identities), str(identities)))
                except CognitiveFaceException as e:
                    rospy.logwarn('identification did not work: %s' %
                                  str(e))
            for f in data:
                face = Face()
                face.faceId = f['faceId']
                if face.faceId in identities:
                    face.person = identities[face.faceId]
                else:
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
                              180.0 * math.pi)
                face.rpy.y = (f['faceAttributes']['headPose']['pitch'] /
                              180.0 * math.pi)
                face.rpy.z = (f['faceAttributes']['headPose']['yaw'] /
                              180.0 * math.pi)
                faces.faces.append(face)
            faces.header = image.header
        except Exception as e:
            rospy.logwarn('failed to detect via the MS Face API: %s' %
                          str(e))
        return faces


    def _identify_autotrainning(self, image):
        
        faces = Faces()
        try:
            data = CF.detect(
                StringIO(self._convert_ros2jpg(image)),
                landmarks=True,
                attributes='age,gender,headPose,smile,glasses')
            identities = []
            
            rospy.loginfo('_identify_autotrainning:: trying to identify persons')
            ids = [f['faceId'] for f in data]
            try:
                identified = CF.identify(ids[0:10], self._person_group_id)
                #print 'identified=',identified
                for i in identified:
                    if len(i['candidates']) > 0:
                        pid = i['candidates'][0]['personId']
                        person = PERSON.get(self._person_group_id, pid)
                        
                        identity={'faceId':i['faceId'],'name':person['name'], 'confidence':i['candidates'][0]['confidence']}
                        
                        identities.append(identity)
                        
                rospy.loginfo('_identify_autotrainning:: identified %d persons in this image: %s '
                              % (len(identities), str(identities)))
            except CognitiveFaceException as e:
                rospy.logwarn('identification did not work: %s' %
                              str(e))
            for f in data:
                face = Face()
                face.faceId = f['faceId']
                
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
                              180.0 * math.pi)
                face.rpy.y = (f['faceAttributes']['headPose']['pitch'] /
                              180.0 * math.pi)
                face.rpy.z = (f['faceAttributes']['headPose']['yaw'] /
                              180.0 * math.pi)
                
                index_identity=None

                for i in range(len(identities)):
                    if face.faceId == identities[i]['faceId']:
                        index_identity=i
                        break
 
                if index_identity!= None:
                    face.person = identities[index_identity]['name']
                    face.confidence = identities[index_identity]['confidence']
                else:
                    
                    
                    persons = PERSON.lists(self._person_group_id)
                    
                    # only add person if the detection is good
                    if self._requeriments_addperson(face):
                        
                        face.person ='user-'+str(len(persons))
                        rospy.loginfo('_identify_autotrainning:: trying to add new person "%s"',face.person )
                                            
                                                
                        self._add_face_target(image,face)
                        
                        face.confidence = -1.0
                 

                faces.faces.append(face)
            faces.header = image.header
        except Exception as e:
            rospy.logwarn('failed to detect via the MS Face API: %s' %
                          str(e))
        return faces
        
    def _add_face_target(self,img_msg, face_req):
        
        person = self._init_person(face_req.person, delete_first=True)
        
        
        pfid = PERSON.add_face(
            StringIO(self._convert_ros2jpg(img_msg)),
            self._person_group_id,
            person['personId'],
            target_face="%d,%d,%d,%d"
            % (face_req.faceRectangle.x_offset,
               face_req.faceRectangle.y_offset,
               face_req.faceRectangle.width,
               face_req.faceRectangle.height))

        face_req.faceId = pfid['persistedFaceId']

        rospy.loginfo('restarting training with new '
                      'face for group "%s".' % self._person_group_id)
        PG.train(self._person_group_id)

        
    def _requeriments_addperson(self, f):
        
        if f.rpy.x > self._add_person_maxroll:
            
            rospy.logwarn(' failed trainning requeriments: Roll (%s) is bigger than limit (%s)' % (str(f.rpy.x),  str(self._add_person_maxroll)) )
            
            return False
        if f.rpy.z > self._add_person_maxyawl:
            
            rospy.logwarn(' failed trainning requeriments: Yaw (%s) is bigger than limit (%s)' % (str(f.rpy.z),  str(self._add_person_maxyawl)) )
            
            return False
        
        area = float(f.faceRectangle.width * f.faceRectangle.height)

        if area < self._add_person_minarea:
            
            rospy.logwarn(' failed trainning requeriments: Area (%s) is lower than limit (%s)' % (str(area),  str(self._add_person_minarea)) )
            
            return False            
        
        return True


    def _delete_person_srv(self,  req):
        gid = self._person_group_id
        #gid=req.id_group
        person = self._find_person_by_name(req.person)
        try:

            if person is not None :
                print('delete existing person "%s"' % req.person)
                PERSON.delete(gid, person['personId'])
                person = None
        except CognitiveFaceException as e:
            print('Operation failed for person "%s".'
                          ' Exception: %s'
                          % (gid, e))

        return []

rospy.init_node('cognitivefaceapi')
cfa = CognitiveFaceROS()
rospy.spin()
