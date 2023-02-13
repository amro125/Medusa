import math
import mediapipe as mp
import numpy as np
import cv2

hand_landmarks = {
    'thumb': [1, 2, 3, 4],
    'indexFinger': [5, 6, 7, 8, 9],
    'middleFinger': [9, 10, 11, 12],
    'thirdFinger': [13, 14, 15, 16],
    'pinkieFinger': [17, 18, 19, 20],
    'palm': [0, 5, 9, 13, 17]
}


def angle(self, point1, point2, point3):
    """ Calculate angle between two lines """
    if point1 == (0, 0) or point2 == (0, 0) or point3 == (0, 0):
        return 0

    numerator = point2[1] * (point1[0] - point3[0]) + point1[1] * (point3[0] - point2[0]) + point3[1] * (
            point2[0] - point1[0])

    denominator = (point2[0] - point1[0]) * (point1[0] - point3[0]) + (point2[1] - point1[1]) * (point1[1] - point3[1])

    try:
        ang = math.atan(numerator / denominator)
        ang = ang * 180 / math.pi
        if ang < 0:
            ang = 180 + ang
        return ang
    except:
        return 90.0


def angle_of_singleline(point1, point2):
    """ Calculate angle of a single line """
    x_diff = point2[0] - point1[0]
    y_diff = point2[1] - point1[1]

    return math.degrees(math.atan2(y_diff / x_diff))


def dist_xy(point1, point2):
    """ Euclidean distance between two points """
    return math.sqrt((point2[0] - point1[0]) ** 2 + (point2[1] - point1[1]) ** 2)


def point_position(self, point, line_pt_1, line_pt_2):
    """
    Left or Right position of the point from a line
    """
    value = (line_pt_2[0] - line_pt_1[0]) * (point[1] - line_pt_1[1]) - (line_pt_2[1] - line_pt_1[1]) * (
            point[0] - line_pt_1[0])

    if value >= 0:
        return "left"

    return "right"


def calc_bounding_rect(image, landmarks):
    """ Calculate bounding rectangle of the hand """
    image_width, image_height = image.shape[1], image.shape[0]

    for _, landmark in enumerate(landmarks.landmark):
        landmark_x = min(int(landmark.x * image_width), image_width - 1)

        landmark_y = min(int(landmark.y * image_height), image_height - 1)

        landmark_point = [np.array((landmark_x, landmark_y))]

        landmark_array = np.append(landmark_array, landmark_point, axis=0)

    x, y, w, h = cv2.boundingRect(landmark_array)

    return [x, y, x + w, y + h]


def detect_hand_gesture(landmarks):
    """ Extract the x and y coordinates of the wrist landmarks """
    point = landmarks.landmark[6].y
    first_finger_is_open = landmarks.landmark[7].y < point and landmarks.landmark[8].y < point

    point = landmarks.landmark[10].y
    second_finger_is_open = landmarks.landmark[11].y < point and landmarks.landmark[12].y < point

    point = landmarks.landmark[14].y
    third_finger_is_open = landmarks.landmark[15].y < point and landmarks.landmark[16].y < point

    point = landmarks.landmark[18].y
    fourth_finger_is_open = landmarks.landmark[19].y < point and landmarks.landmark[20].y < point

    if first_finger_is_open or second_finger_is_open or third_finger_is_open or fourth_finger_is_open:
        # return "Hand Open"
        return 1
    else:
        # return "Hand Close"
        return 2

def detectUpright(hand_landmarks):
    pointerDistance = math.sqrt((hand_landmarks.landmark[8].x - hand_landmarks.landmark[5].x)**2 + (hand_landmarks.landmark[8].y - hand_landmarks.landmark[5].y)**2)
    middleDistance = math.sqrt((hand_landmarks.landmark[12].x - hand_landmarks.landmark[9].x)**2 + (hand_landmarks.landmark[12].y - hand_landmarks.landmark[9].y)**2)
    ringDistance = math.sqrt((hand_landmarks.landmark[16].x - hand_landmarks.landmark[13].x)**2 + (hand_landmarks.landmark[16].y - hand_landmarks.landmark[13].y)**2)

    pointerStraight = hand_landmarks.landmark[8].y < abs(hand_landmarks.landmark[5].y - pointerDistance*4/5)
    middleStraight = hand_landmarks.landmark[12].y < abs(hand_landmarks.landmark[9].y - middleDistance*4/5)
    ringStraight = hand_landmarks.landmark[16].y < abs(hand_landmarks.landmark[13].y - ringDistance*4/5)

    if (pointerStraight and middleStraight and ringStraight):
        return True
    else:
        return False

def detectFront(hand_landmarks):
    if (hand_landmarks.landmark[17].x > hand_landmarks.landmark[13].x > hand_landmarks.landmark[9].x > hand_landmarks.landmark[5].x) and (hand_landmarks.landmark[18].x > hand_landmarks.landmark[14].x > hand_landmarks.landmark[10].x > hand_landmarks.landmark[6].x) and hand_landmarks.landmark[19].x > hand_landmarks.landmark[15].x > hand_landmarks.landmark[11].x > hand_landmarks.landmark[7].x and hand_landmarks.landmark[20].x > hand_landmarks.landmark[16].x > hand_landmarks.landmark[12].x > hand_landmarks.landmark[8].x:
        return True
    else:
        return False
