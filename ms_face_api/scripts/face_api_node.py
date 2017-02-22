import cognitive_face as CF

KEY = 'be062e88698e4777ac6196623d7230dd'  # Replace with a valid Subscription Key here.
CF.Key.set(KEY)

img_url = 'https://raw.githubusercontent.com/Microsoft/Cognitive-Face-Windows/master/Data/detection1.jpg'
result = CF.face.detect(img_url, landmarks=True, attributes='age,gender,headPose,smile,facialHair,glasses')
print result
