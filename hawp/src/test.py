import torch
import parsing
import time
from parsing.config import cfg
from parsing.utils.comm import to_device
from parsing.dataset.build import build_transform
from parsing.dataset import build_test_dataset
from parsing.detector import WireframeDetector
from parsing.utils.logger import setup_logger
from parsing.utils.metric_logger import MetricLogger
from parsing.utils.miscellaneous import save_config
from parsing.utils.checkpoint import DetectronCheckpointer
import torch.utils.data as Data
from skimage import io
from skimage import color
import numpy as np
import os
import os.path as osp
import time
import datetime
import argparse
import logging
import matplotlib.pyplot as plt
from tqdm import tqdm
import json
import cv2 
parser = argparse.ArgumentParser(description='HAWP Testing')

parser.add_argument("--config-file",
                    metavar="FILE",
                    help="path to config file",
                    type=str,
                    default="/home/lihao/ros/slam/src/hawp_py2/src/config-files/hawp.yaml"
                    # required=True,
                    )

parser.add_argument("--img",type=str,default="/home/lihao/ros/slam/src/hawp/src/image/image7.jpg",
                    help="image path")                    

parser.add_argument("--threshold",
                    type=float,
                    default=0.95)

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
    print("image_tensor:",image_tensor.shape)
    meta = {
        'filename': impath,
        'height': image.shape[0],
        'width': image.shape[1],
    }
    print(image.shape[0])
    print(image.shape[1])
    
    checkpointer = DetectronCheckpointer(cfg,
                                         model,
                                         save_dir=cfg.OUTPUT_DIR,
                                         save_to_disk=True,
                                         logger=logger)
    _ = checkpointer.load()
    model = model.eval()
    # print(model)
    start_time = time.time()
    with torch.no_grad():
        output, _ = model(image_tensor,[meta])
        output = to_device(output,'cpu')
    # print(output)
    
    lines = output['lines_pred'].numpy()
    scores = output['lines_score'].numpy()
    end_time = time.time()
    run_time = end_time - start_time
    print(run_time)
    idx = scores>args.threshold

    # results = []
    # for k in output.keys():
    #     if isinstance(output[k], torch.Tensor):
    #             output[k] = output[k].tolist()
    #     results.append(output)

    # output_path = '/home/lihao/dataset/SLAM/FE-Wireframe/images-lines/lsd'
    # with open(os.path.join(output_path, 'result2.json'), 'w') as f:
    #     json.dump(results, f)

    plt.figure(figsize=(6,6))    
    plt.subplots_adjust(top = 1, bottom = 0, right = 1, left = 0, 
            hspace = 0, wspace = 0)
    plt.imshow(image)
    plt.plot([lines[idx,0],lines[idx,2]],
                        [lines[idx,1],lines[idx,3]], 'b-')
    plt.plot(lines[idx,0],lines[idx,1],'c.')                        
    plt.plot(lines[idx,2],lines[idx,3],'c.')                        
    plt.axis('off')
    plt.show()

def my_test(cfg, impath,results,model):
    

    transform = build_transform(cfg)
    image = cv2.imread(impath)
    print(image.shape, image.dtype)
    
    if len(image.shape) == 2:  # gray image
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    image_tensor = transform(image.astype(float))[None].to(device)
    print("image_tensor:",image_tensor.shape)
    meta = {
        'filename': impath,
        'height': image.shape[0],
        'width': image.shape[1],
    }
    print(image.shape[0])
    print(image.shape[1])
    

    # print(model)
    start_time = time.time()
    with torch.no_grad():
        output, _ = model(image_tensor,[meta])
        output = to_device(output,'cpu')
    # print(output)
    
    # lines = output['lines_pred'].numpy()
    # scores = output['lines_score'].numpy()
    # end_time = time.time()
    # run_time = end_time - start_time
    # print(run_time)
    # idx = scores>args.threshold
    # if len(output['lines_pred']):
    #     line_pred = output['lines_pred'].numpy()
    #     line_score = output['lines_score'].numpy()
    #     filename = output['filename']

    #     mask = line_score > args.threshold
    #     line_pred = line_pred[mask]

    # results = []
    # result = {}
    for k in output.keys():
        if isinstance(output[k], torch.Tensor):
                output[k] = output[k].tolist()
    results.append(output)

def my_test2(cfg,model,output_path):
    logger = logging.getLogger("hawp.testing")
    device = cfg.MODEL.DEVICE

    test_datasets = build_test_dataset(cfg)
    
    output_dir = output_path

    for name, dataset in test_datasets:
        results = []
        logger.info('Testing on {} dataset'.format(name))
        if format(name)=='wireframe_test':
            for i, (images, annotations) in enumerate(tqdm(dataset)):
                with torch.no_grad():
                    output, extra_info = model(images.to(device), annotations)
                    output = to_device(output,'cpu')

                for k in output.keys():
                    if isinstance(output[k], torch.Tensor):
                        output[k] = output[k].tolist()
                results.append(output)
            outpath_dataset = osp.join(output_dir,'{}.json'.format(name))
            logger.info('Writing the results of the {} dataset into {}'.format(name,
                        outpath_dataset))
            with open(os.path.join(output_path, 'result3.json'), 'w') as f:
                json.dump(results,f)
    
if __name__ == "__main__":

    cfg.merge_from_file(args.config_file)
    cfg.freeze()
    
    output_dir = cfg.OUTPUT_DIR
    logger = setup_logger('hawp', output_dir)
    logger.info(args)
    logger.info("Loaded configuration file {}".format(args.config_file))


    # test(cfg,args.img)
    evaluate = True
    overwirte = True
    results = []
    detect = True
    dataset_path = '/home/lihao/dataset/SLAM/FE-Wireframe'
    image_path = '/home/lihao/dataset/SLAM/FE-Wireframe/images-blur'
    output_path = '/home/lihao/dataset/SLAM/FE-Wireframe/images-lines/lsd'

    logger = logging.getLogger("hawp.testing")
    device = cfg.MODEL.DEVICE
    model = WireframeDetector(cfg)
    model = model.to(device)

    checkpointer = DetectronCheckpointer(cfg,
                                         model,
                                         save_dir=cfg.OUTPUT_DIR,
                                         save_to_disk=True,
                                         logger=logger)
    _ = checkpointer.load()
    model = model.eval()

    i=0
    # if detect:
    #     gt_file = os.path.join(dataset_path, 'test.json')
    #     with open(gt_file, 'r') as f:
    #         gt_annotations = json.load(f)
    #     for gt_annin in gt_annotations:
    #         i = i +1
    #         image_name = gt_annin['filename']
    #         image_name = os.path.join(image_path,image_name)
    #         image_name_png = gt_annin['filename']
    #         my_test(cfg,image_name,results,model)
    #         print(i)

    # if overwirte:
    #     with open(os.path.join(output_path, 'result2.json'), 'w') as f:
    #         json.dump(results, f)

    my_test2(cfg,model,output_path)

