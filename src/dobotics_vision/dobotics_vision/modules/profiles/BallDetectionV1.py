import cv2
import numpy as np
from typing import TypedDict


class BallDetectionV1Return(TypedDict):
    image: np.ndarray
    is_detected: bool
    ball_img_x: int
    ball_img_y: int
    ball_radius: int


def BallDetectionV1(img: np.ndarray) -> BallDetectionV1Return:
    img_grayblur = cv2.GaussianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (7, 7), 0)
    img_edge = cv2.Canny(img_grayblur, threshold1=100, threshold2=200)

    return {
        'image': img_edge,
        'is_detected': False,
        'ball_img_x': 0,
        'ball_img_y': 0,
        'ball_radius': 0
    }