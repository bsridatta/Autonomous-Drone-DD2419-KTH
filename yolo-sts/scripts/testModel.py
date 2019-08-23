from net.build import TFNet
import cv2


options = {"model": "/media/datta/Sri Datta/dd2419_ws/src/yolo-sts/scripts/cfg/tiny-yolo-voc-sts.cfg", 
            "pbLoad": "/media/datta/Sri Datta/dd2419_ws/src/yolo-sts/scripts/built_graph/tiny-yolo-voc-sts.pb",
            "metaLoad":"/media/datta/Sri Datta/dd2419_ws/src/yolo-sts/scripts/built_graph/tiny-yolo-voc-sts.meta",
            "backup":"/media/datta/Sri Datta/dd2419_ws/src/yolo-sts/scripts/ckpt/",
            "load":-1,
            "threshold":0.8               
        }	

tfnet = TFNet(options)

imgcv = cv2.imread("sample.jpg")
result = tfnet.return_predict(imgcv[None, :, :, :])
print(result)
