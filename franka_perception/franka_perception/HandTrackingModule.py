import cv2
import mediapipe as mp
import time
import numpy as np


class HandDetector:
    def __init__(self, mode=False, max_hands=2, complexity=1, detection_con=0.5, track_con=0.5):
        self.mode = mode
        self.max_hands = max_hands
        self.complexity = complexity
        self.detection_con = detection_con
        self.track_con = track_con

        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.max_hands, self.complexity, self.detection_con, self.track_con)
        self.mpDraw = mp.solutions.drawing_utils
        
        self.results = None
        
    def find_hands(self, img, draw=True):
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(img_rgb)
        # print(results.multi_hand_landmarks)

        if self.results.multi_hand_landmarks:
            # print(self.results.multi_hand_landmarks)
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(img, handLms, self.mpHands.HAND_CONNECTIONS)
        return img

    def find_position(self, img, hand_no=0, draw=True):

        lmlist = []
        if self.results.multi_hand_landmarks:
            my_hand = self.results.multi_hand_landmarks[hand_no]
            for id, lm in enumerate(my_hand.landmark):
                h, w, c = img.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                lmlist.append([id, cx, cy])
                if draw:
                    cv2.circle(img, (cx, cy), 7, (255, 0, 255), cv2.FILLED)
        return lmlist


def post_process(image: np.ndarray, lmlist):
    img = image.copy()
    # draw bounding box
    if len(lmlist) > 4:
        lmlist = np.array(lmlist)
        # print(lmlist.shape)
        b_x_1 = np.min(lmlist[:, 1])
        b_x_2 = np.max(lmlist[:, 1])
        b_y_1 = np.min(lmlist[:, 2])
        b_y_2 = np.max(lmlist[:, 2])
        cv2.rectangle(img, (b_x_1, b_y_1), (b_x_2, b_y_2), (0, 255, 0), 2)
        bbox = [b_x_1, b_x_2, b_y_1, b_y_2]
    else:
        bbox = []
    return img, bbox


def main():
    pTime = 0
    cTime = 0
    # cap = cv2.VideoCapture(0)
    detector = HandDetector()


    img = cv2.imread("00066.jpg")
    img = detector.find_hands(img)

    # lmlist is the landmarks list
    lmlist = detector.find_position(img)

    post_process(img, lmlist)

    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime

    cv2.putText(img, str(int(fps)), (10, 70), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)

    cv2.imshow("Image", img)
    cv2.waitKey(0)


if __name__ == "__main__":
    main()
