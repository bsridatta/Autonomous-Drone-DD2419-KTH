#!/usr/bin/env python

from net.build import TFNet
import cv2
import rospy
from tlight_node import TLightNode
import argparse


def process(model, img):
    # print('**in returning prediction**')
    result = model.return_predict(img[None, :, :, :])

    return result

def getModel():
    # print('***in get model***')
    # options = {"model": "/media/datta/Sri Datta/dd2419_ws/src/yolo-sts/scripts/cfg/tiny-yolo-voc-sts.cfg", 
    #             "pbLoad": "/media/datta/Sri Datta/dd2419_ws/src/yolo-sts/scripts/built_graph/tiny-yolo-voc-sts.pb",
    #             "metaLoad":"/media/datta/Sri Datta/dd2419_ws/src/yolo-sts/scripts/built_graph/tiny-yolo-voc-sts.meta",
    #             "load":"/media/datta/Sri Datta/dd2419_ws/src/yolo-sts/scripts/built_graph",
    #             "threshold":"0.1"
    #             }

    options = {"model": "/media/datta/Sri Datta/dd2419_ws/src/yolo-sts/scripts/cfg/tiny-yolo-voc-sts.cfg", 
                "pbLoad": "/media/datta/Sri Datta/dd2419_ws/src/yolo-sts/scripts/built_graph/tiny-yolo-voc-sts.pb",
                "metaLoad":"/media/datta/Sri Datta/dd2419_ws/src/yolo-sts/scripts/built_graph/tiny-yolo-voc-sts.meta",
                "backup":"/media/datta/Sri Datta/dd2419_ws/src/yolo-sts/scripts/ckpt/",
                "load":-1,
                "threshold":0.8              
            }

    model = TFNet(options)
    print('***done loading***')
    return model

def main():
    parser = argparse.ArgumentParser(description='Script for running yolo_light node')
    args = parser.parse_args()
    node = TLightNode(lambda: getModel(), process)
    rospy.spin()

if __name__ == '__main__':
    main()
