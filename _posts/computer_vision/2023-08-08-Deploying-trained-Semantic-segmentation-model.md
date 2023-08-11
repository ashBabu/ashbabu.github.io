---
layout: post
title: "Part 03 : Deploying a trained Semantic segmentation model"
date: 2023-08-08 20:22:00-0500
description: 
# youtubeId: 90K8CWjifUs
categories: Computer-Vision
---

This article builds upon the previous two post, viz., [training](/blog/2023/Semantic-segmentation/), [inference](/blog/2023/Inference-on-trained-Semantic-segmentation-model/) and shows how a live video stream from intel realsense camera could be used to do inference. This only gives a basic idea and there are other methods like `tensorrt`, `torchscript`, `onnx` etc for faster inference.


The implementation uses ROS but it can be done otherwise as well using the realsense SDK

```bash
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np
import utils_fn as util
from lightningModel import OurModel

class Deploy(object):
    def __init__(self, modelPath="chk_pts/trained_cityscapes_final.pth", lightning=False):
        # Params
        self.image = None
        self.lightning = lightning
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)
        self.modelPath = modelPath
        self.loadModel(self.modelPath, lightning=self.lightning)
        self.imgPath = "../datasets/cityscapes/leftImg8bit/img.png"
        # Subscribers
        rospy.Subscriber("/camera/color/image_raw", Image, self.callback)

    def callback(self, msg):
        if msg is not None:
            rospy.loginfo('Image received...')
            self.image = self.br.imgmsg_to_cv2(msg)

    def loadModel(self, modelPath=None, lightning=False):
        if lightning:
            self.model = OurModel(n_classes=20)
            self.model.load_state_dict(torch.load('chk_pts/model.pth'))
        elif modelPath is not None:
            self.model = torch.load(modelPath)["model"]
        else:
            self.model = torch.load(self.modelPath)["model"]

        self.model.to(util.device)
        self.model.eval()

    def getPrediction(self, img_tensor):
        with torch.no_grad():
            prediction = self.model(img_tensor.to(util.device)).squeeze(0)
        return prediction.detach().cpu()

    def postProcess(self, prediction):
        t = torch.argmax(prediction, 0)
        decoded_output = util.decode_segmap(t)
        return cv2.cvtColor(decoded_output.astype(np.float32), cv2.COLOR_RGB2BGR)

    def showImage(self, img):
        cv2.namedWindow("Image window", cv2.WINDOW_NORMAL)
        cv2.imshow("Image window", img)
        cv2.waitKey(3)

    def start(self):
        while not rospy.is_shutdown():
            if self.image is not None:
                imgCV2 = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
                img_tensor = util.transform(image=imgCV2)["image"].unsqueeze(0)
                invimg = util.inv_normalize(img_tensor).squeeze(0)
                predicted = self.getPrediction(img_tensor)
                processed_image = self.postProcess(predicted)
                numpy_horizontal = np.hstack((np.moveaxis(invimg.numpy(), 0, 2), processed_image))
                self.showImage(numpy_horizontal)
            self.loop_rate.sleep()


if __name__ == '__main__':
    rospy.init_node("deploy", anonymous=True)
    modelPath = "chk_pts/hrnet_latest.pth"
    my_node = Deploy(modelPath=modelPath, lightning=True)
    my_node.start()
print("done")
```

### Explanation
The `cv_bridge` package is used to convert `sensor_msgs/Image` to `opencv` format. This converted image is sent to the model for performing inference.