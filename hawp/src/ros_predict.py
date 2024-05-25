#!/usr/bin/env python3
import rospy
import torch
import parsing
import time
import datetime
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from hawp.msg import lines2d
import cv2
import numpy as np
import logging
from parsing.config import cfg
from parsing.utils.comm import to_device
from parsing.dataset.build import build_transform
from parsing.detector import WireframeDetector
from parsing.utils.logger import setup_logger
from parsing.utils.metric_logger import MetricLogger
from parsing.utils.miscellaneous import save_config
from parsing.utils.checkpoint import DetectronCheckpointer



threshold = 0.83

class Nodo(object):
    def __init__(self, cfg):
        # Params
        self.image = None
        self.header = None
        self.br = CvBridge()
        self.msg_list = []
        self.pub_img_set = True
        self.image_topic = cfg.IMAGE_TOPIC

        self.logger = logging.getLogger("hawp.testing")
        self.device = cfg.MODEL.DEVICE
        self.model = WireframeDetector(cfg)
        self.model = self.model.to(self.device)
        self.loop_rate = rospy.Rate(200)

        # Publishers
        self.pub = rospy.Publisher('/hawp/Lines2d', lines2d, queue_size=1000)
        self.pub_image = rospy.Publisher('/hawp/feature_image', Image, queue_size=1000)

        # Subscribers
        rospy.Subscriber(self.image_topic, Image, self.callback)

        # camera parameters

        #hawp
        checkpointer = DetectronCheckpointer(cfg,
                                         self.model,
                                         save_dir=cfg.OUTPUT_DIR,
                                         save_to_disk=False,
                                         logger=self.logger)
        _ = checkpointer.load()
        self.transform = build_transform(cfg)
        self.model = self.model.eval()

    def callback(self, msg):
        self.msg_list.append(msg)

    def start(self):
        pre_msg_time = rospy.Time(0)
        while not rospy.is_shutdown():
            if len(self.msg_list):
                self.image = self.br.imgmsg_to_cv2(self.msg_list[-1])
                img_msg = self.msg_list[-1]
                self.msg_list.pop()
                self.header = img_msg.header
            if self.image is not None:
                msg_time = self.header.stamp
                if msg_time > pre_msg_time:
                    pre_msg_time = msg_time
                    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
                    res_img = clahe.apply(self.image.copy())
                    if len(self.image.shape) == 2:  # gray image
                        img = cv2.cvtColor(res_img.copy(), cv2.COLOR_GRAY2BGR)
                        img_tensor = self.transform(img.astype(float))[None].to(self.device)
                    else:
                        img = res_img.copy()
                        img_tensor = self.transform(img.astype(float))[None].to(self.device)
                    meta = {
                        'filename': 'impath',
                        'height': img.shape[0],
                        'width': img.shape[1],
                    }
                    with torch.no_grad():
                        output, _ = self.model(img_tensor, [meta])
                        output = to_device(output, 'cpu')
                    lines = output['lines_pred'].numpy()
                    scores = output['lines_score'].numpy()
                    idx = scores > threshold
                    lines=lines[idx,:]
                    lines2d_msg = lines2d(
                        header=self.header, startx=lines[:, 0], starty=lines[:, 1], endx=lines[:, 2], endy=lines[:, 3])
                    self.pub.publish(lines2d_msg)
                    self.pub_image.publish(img_msg)
            self.loop_rate.sleep()

    
if __name__ == "__main__":

    rospy.init_node('hawp')

    config_file=rospy.get_param('~config_file')
    cfg.merge_from_file(config_file)
    cfg.freeze()
    
    # print(cfg)
    output_dir = cfg.OUTPUT_DIR
    logger = setup_logger('hawp', output_dir)
    # logger.info(args)
    logger.info("Loaded configuration file {}".format(config_file))

    my_node=Nodo(cfg)
    my_node.start()

