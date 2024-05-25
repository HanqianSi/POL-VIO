import torch
import parsing
from parsing.config import cfg
from parsing.utils.comm import to_device
from parsing.dataset.build import build_transform
from parsing.detector import WireframeDetector
from parsing.utils.logger import setup_logger
from parsing.utils.metric_logger import MetricLogger
from parsing.utils.miscellaneous import save_config
from parsing.utils.checkpoint import DetectronCheckpointer
from skimage import io
from skimage import color
import os
import os.path as osp
import time
import datetime
import argparse
import logging
import matplotlib.pyplot as plt
from tqdm import tqdm
import numpy as np
import json
import cv2 
parser = argparse.ArgumentParser(description='HAWP Testing')

parser.add_argument("--config-file",
                    metavar="FILE",
                    help="path to config file",
                    type=str,
                    required=True,
                    )

parser.add_argument("--img",type=str,required=True,
                    help="image path")                    

parser.add_argument("--threshold",
                    type=float,
                    default=0.85)

args = parser.parse_args()

def test(cfg, impath):
    logger = logging.getLogger("hawp.testing")
    device = cfg.MODEL.DEVICE
    model = WireframeDetector(cfg)
    model = model.to(device)

    transform = build_transform(cfg)
    image = cv2.imread(impath)
    print(image.shape, image.dtype)
    if len(image.shape) == 2:  # gray image
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    image_tensor = transform(image.astype(float))[None].to(device)
    meta = {
        'filename': impath,
        'height': image.shape[0],
        'width': image.shape[1],
    }
    
    checkpointer = DetectronCheckpointer(cfg,
                                         model,
                                         save_dir=cfg.OUTPUT_DIR,
                                         save_to_disk=True,
                                         logger=logger)
    _ = checkpointer.load()
    model = model.eval()

    start_time = time.time()

    with torch.no_grad():
        output, _ = model(image_tensor,[meta])
        output = to_device(output,'cpu')

    
    lines = output['lines_pred'].numpy()
    scores = output['lines_score'].numpy()
    idx = scores>args.threshold
    # print(lines.shape())
    lines = lines[idx,:]
    # print(lines.shape())
    print(lines)
    np.savetxt("data.txt",lines,delimiter=',',fmt='%.08f')
    

    print(time.time() - start_time)

    plt.figure(figsize=(6,6))    
    plt.subplots_adjust(top = 1, bottom = 0, right = 1, left = 0, 
            hspace = 0, wspace = 0)
    plt.imshow(image)
    plt.plot([lines[:,0],lines[:,2]],
                        [lines[:,1],lines[:,3]], 'b-')
    plt.plot(lines[:,0],lines[:,1],'c.')                        
    plt.plot(lines[:,2],lines[:,3],'c.')                        
    plt.axis('off')
    plt.show()
    
if __name__ == "__main__":

    cfg.merge_from_file(args.config_file)
    cfg.freeze()
    
    output_dir = cfg.OUTPUT_DIR
    logger = setup_logger('hawp', output_dir)
    logger.info(args)
    logger.info("Loaded configuration file {}".format(args.config_file))


    test(cfg,args.img)

