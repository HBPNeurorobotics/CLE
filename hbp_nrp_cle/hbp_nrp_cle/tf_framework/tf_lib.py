"""
Contains a library of transfer functions
"""

# import sensor_msgs.msg
from cv_bridge import CvBridge

__author__ = 'GeorgHinkel'

bridge = CvBridge()


def detect_red(image):
    """
    Detects a red image
    :param image: The image
    """
    # assert isinstance(image, sensor_msgs.msg.Image)
    red_left_rate = 0.
    red_right_rate = 0.
    green_blue_rate = 0.
    if not isinstance(image, type(None)):  # Problem: starts as NoneType
        # print eye_sensor.changed
        cv_image = bridge.imgmsg_to_cv2(image, "rgb8") / 256.
        cv_image = 5000 ** cv_image / 5000

        for i in range(0, cv_image.shape[0]):
            for j in range(0, cv_image.shape[1]):
                r = cv_image[i, j, 0]
                g = cv_image[i, j, 1]
                b = cv_image[i, j, 2]

                if r > 2 * (g + b):
                    if j < cv_image.shape[1] / 2:
                        red_left_rate += 1.
                    else:
                        red_right_rate += 1.
                else:
                    green_blue_rate += 1.

        red_left_rate *= (6. / cv_image.size)
        red_right_rate *= (6. / cv_image.size)
        green_blue_rate *= (3. / cv_image.size)

        print "red_left_rate: ", red_left_rate
        print "red_right_rate: ", red_right_rate
        print "green_blue_rate: ", green_blue_rate

#        for i in range(0, cv_image.shape[0]):
#            for j in range(0, cv_image.shape[1]):
#                if j < cv_image.shape[1] / 2:
#                    red_left_rate += cv_image[i, j, 0]
#                else:
#                    red_right_rate += cv_image[i, j, 0]
#                green_blue_rate += cv_image[i, j, 1]
#                green_blue_rate += cv_image[i, j, 2]
#        red_left_rate *= (1. / cv_image.size)
#        red_right_rate *= (1. / cv_image.size)
#        green_blue_rate *= (1. / cv_image.size)

    class __results(object):
        """
        An intermediate helper class for the results of detect_red
        """
        def __init__(self, left, right, go_on):
            self.left = left
            self.right = right
            self.go_on = go_on

    return __results(red_left_rate, red_right_rate, green_blue_rate)
