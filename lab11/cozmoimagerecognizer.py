#!/usr/bin/env python

# Adam Heaney

from imgclassification import ImageClassifier
from skimage import exposure
import cozmo
import sys
import numpy as np

NONE_CLASS = 'none'
IMG_RECOGNITION_HISTORY = 5
IMG_RECOGNITION_MIN_CONFIDENCE = 0.75

# compute the most common value in a list and return its count
def computeMostFrequentValue(listOfValues):
    valueCounts = { }
    for value in listOfValues:
        if value in valueCounts:
            valueCounts[value] = valueCounts[value] + 1
        else:
            valueCounts[value] = 1

    mostFrequentValue = None
    mostFrequentValueCount = 0
    for value, count in valueCounts.items():
        if count > mostFrequentValueCount:
            mostFrequentValue = value
            mostFrequentValueCount = count

    return value, count

# state where the robot will continuously process images, looking for known images
async def detectKnownObject(robot: cozmo.robot.Robot, classifier: ImageClassifier):
    pastImages = []
    while True:
        #get camera image
        event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)

        cozmoImage = np.asarray(event.image)
        cozmoImage = exposure.rescale_intensity(cozmoImage, (85, 255))

        imageFeatures = classifier.extract_image_features([cozmoImage])
        detectedImageClass = classifier.predict_labels(imageFeatures)[0]

        pastImages.append(detectedImageClass)
        while len(pastImages) > IMG_RECOGNITION_HISTORY:
            pastImages.pop(0)

        if len(pastImages) < IMG_RECOGNITION_HISTORY:
            continue

        mostCommonImageClass, mostCommonImageClassCount = computeMostFrequentValue(pastImages)
        confidence = mostCommonImageClassCount / IMG_RECOGNITION_HISTORY

        if confidence > IMG_RECOGNITION_MIN_CONFIDENCE and not mostCommonImageClass == NONE_CLASS:
            return mostCommonImageClass

# state where the robot will announce the image that it detected and play an animation
async def announceDetectedImage(robot: cozmo.robot.Robot, detectedImageClass):
    print("cozmo detected ", detectedImageClass)
    await robot.play_anim_trigger(cozmo.anim.Triggers.AcknowledgeObject).wait_for_completed()
    await robot.say_text(detectedImageClass, play_excited_animation=False, use_cozmo_voice=True).wait_for_completed()

async def run(robot: cozmo.robot.Robot):
    print("training classifier...")

    img_clf = ImageClassifier()

    # load images
    (train_raw, train_labels) = img_clf.load_data_from_folder('./train/')
    (test_raw, test_labels) = img_clf.load_data_from_folder('./test/')
    
    # convert images into features
    train_data = img_clf.extract_image_features(train_raw)
    test_data = img_clf.extract_image_features(test_raw)
    
    # train model and test on training data
    img_clf.train_classifier(train_data, train_labels)

    print("training completed.")

    await robot.set_head_angle(cozmo.robot.MIN_HEAD_ANGLE + cozmo.util.degrees(30.0)).wait_for_completed()

    robot.camera.enable_auto_exposure()

    try:
        while True:
            detectedMarker = await detectKnownObject(robot, img_clf)
            await announceDetectedImage(robot, detectedMarker)

    except KeyboardInterrupt:
        print("")
        print("Exit requested by user")
    except cozmo.RobotBusy as e:
        print(e)
    except:
        print("Unexpected error:", sys.exc_info()[0])
        raise

if __name__ == '__main__':
    cozmo.run_program(run, use_viewer = True)