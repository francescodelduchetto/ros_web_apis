^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ms_face_api
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.3 (2017-05-26)
------------------
* added confidence result to identify service 
* autotraining mode with params:
	'~autotraining_mode' (bool)
        '~add_person_minarea' (float)
        '~add_person_maxroll' (float)
        '~add_person_maxyawl' (float)
* AddIdentityFace.srv  modified (target added)
* modified PersonGroupSelect.srv  (return known faces list)
* added delete person service 
* Contributors: Roberto Pinillos

0.0.2 (2017-03-15)
------------------
* made the disable warning optional
* Contributors: Marc Hanheide

0.0.1 (2017-03-14)
------------------
* minor fix
* package.xml sanitised
* identification kind of works
* person group management added
* added launch prefix and permission
* some base for identification and better expection handling
* prettier
* made library self-contained
* detect service is working well
* added service
* first time working with ROS
* initial commit
* Contributors: Marc Hanheide
