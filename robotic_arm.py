import math
import cv2
import mediapipe as mp
import serial


def point_rectangle(px, py, rx, ry, rw, rh) -> bool:
    if rx <= px <= rx + rw and ry <= py <= ry + rh:
        return True
    return False


def clamp(n, smallest, largest):
    return max(smallest, min(n, largest))


def map_coordinates(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class Point2D:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y


class ServoCoords:
    def __init__(self, s1=90, s2=90, s3=90):
        self.s1 = s1
        self.s2 = s2
        self.s3 = s3

    def __add__(self, other):
        return ServoCoords(self.s1 + other.s1, self.s2 + other.s2, self.s3 + other.s3)

    def __truediv__(self, other):
        return ServoCoords(self.s1 / other,
                           self.s2 / other,
                           self.s3 / other)

    def clamp_all(self):
        self.s1 = clamp(self.s1, 0, 180)
        self.s2 = clamp(self.s2, 0, 180)
        self.s3 = clamp(self.s3, 0, 180)


def robot_arm():
    # Webcam record
    video_capture = cv2.VideoCapture(0)

    # Video Playback Resolution
    image_dimensions = Point2D(1280, 720)
    video_capture.set(3, image_dimensions.x)
    video_capture.set(4, image_dimensions.y)

    # Hand rectangle
    hand_rect_pos = Point2D(448, 144)
    hand_rect_dim = Point2D(384, 432)

    init_iter = 0

    # Servo Coordinates
    servo_coords = ServoCoords()

    # Hand Detection
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.5)

    # Hand Drawer
    mp_draw = mp.solutions.drawing_utils

    setup = False

    # Connect to Arduino
    arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200)
    # Wait for Arduino to be ready
    print(arduino.readline())

    iterations = 0

    servo_sum = ServoCoords(0, 0, 0)
    max_iter = 8

    while True:
        success, image = video_capture.read()
        distance = 0
        x1 = 0
        y1 = 0
        x2 = 0
        y2 = 0

        landmarks = []
        draw_hand_landmarks(hands, image, image_dimensions, landmarks, mp_draw, mp_hands)

        # If hand was not detected
        if not landmarks:
            setup = False
            init_iter = 0
        else:
            x1, y1 = landmarks[5][1], landmarks[5][2]
            x2, y2 = landmarks[17][1], landmarks[17][2]
            if x1 > x2:
                x1, x2 = x2, x1
            if y1 > y2:
                y1, y2 = y2, y1
            if x1 <= 0 or x2 >= image_dimensions.x or y1 <= 0 or y2 >= image_dimensions.y:
                setup = False
                init_iter = 0
            distance = math.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)

        if not setup:
            cv2.rectangle(image,
                          (hand_rect_pos.x, hand_rect_pos.y),
                          (hand_rect_pos.x + hand_rect_dim.x, hand_rect_pos.y + hand_rect_dim.y),
                          (255, 0, 0),
                          2)
            cv2.putText(image,
                        f"Place your hand in the rectangle",
                        (hand_rect_pos.x - 160, hand_rect_pos.y - 30),
                        cv2.FONT_HERSHEY_TRIPLEX,
                        1.25,
                        (200, 0, 50),
                        3)
            if point_rectangle(x1, y1, hand_rect_pos.x, hand_rect_pos.y, hand_rect_dim.x,
                               hand_rect_dim.y) and point_rectangle(
                x2, y2, hand_rect_pos.x, hand_rect_pos.y, hand_rect_dim.x, hand_rect_dim.y):

                init_iter += 1
                if init_iter == 10:
                    setup = True

        else:
            servo_coords = get_servo_coords(distance, image_dimensions, x1, x2, y1, y2)
            # print(f"servo_coords.s1: {servo_coords.s1}, servo_coords.s2: {servo_coords.s2}, servo_coords.s3: {servo_coords.s3}")

        servo_sum = servo_sum + servo_coords
        # print(f"servo_sum.s1: {servo_sum.s1}, servo_sum.s2: {servo_sum.s2}, servo_sum.s3: {servo_sum.s3}")

        iterations = iterations + 1
        if iterations == max_iter:
            servo_sum /= max_iter

            print(bytes(str(servo_coords.s1) + " " + str(servo_coords.s2) + " " + str(servo_coords.s3), 'utf-8'))
            arduino.write(bytes(str(servo_coords.s1) + " " + str(servo_coords.s2) + " " + str(servo_coords.s3), 'utf-8'))
            servo_sum = ServoCoords(0, 0, 0)
        iterations = iterations % max_iter

        cv2.imshow('Image', image)
        if cv2.waitKey(1) & 0xff == ord('q'):
            break


def get_servo_coords(distance, image_dimensions, x1, x2, y1, y2):
    dif_x = (x2 - x1)
    servo1 = (x1 + x2) / 2
    servo1 = int(map_coordinates(servo1, dif_x, image_dimensions.x - dif_x, 0, 180))

    dif_y = (y2 - y1)
    servo2 = (y1 + y2) / 2
    servo2 = int(map_coordinates(servo2, dif_y, image_dimensions.y - dif_y, 0, 180))

    servo3 = int(map_coordinates(distance, 100, 250, 0, 180))

    servo_coords = ServoCoords(servo1, servo2, servo3)
    servo_coords.clamp_all()
    return servo_coords


def draw_hand_landmarks(hands, image, image_dimensions, landmarks, mp_draw, mp_hands):
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    hand_process = hands.process(rgb_image)
    if hand_process.multi_hand_landmarks:
        hand_landmarks = hand_process.multi_hand_landmarks[0]
        for id, landmark in enumerate(hand_landmarks.landmark):
            x_pixels = int(landmark.x * image_dimensions.x)
            y_pixels = int(landmark.y * image_dimensions.y)
            landmarks.append([id, x_pixels, y_pixels])
        mp_draw.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)


if __name__ == '__main__':
    robot_arm()
