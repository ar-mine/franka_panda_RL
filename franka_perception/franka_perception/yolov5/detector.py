import os
import platform
import sys
from pathlib import Path

import numpy as np
import torch

ROOT = "/home/armine/ROS2/franka_ws/src/franka_panda_RL/franka_perception/franka_perception/yolov5/"  # relative
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

from models.common import DetectMultiBackend
from utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadScreenshots, LoadStreams
from utils.general import (LOGGER, Profile, check_file, check_img_size, check_imshow, check_requirements, colorstr, cv2,
                           increment_path, non_max_suppression, print_args, scale_boxes, strip_optimizer, xyxy2xywh)
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import select_device, smart_inference_mode
from utils.augmentations import letterbox


class YoloV5:
    def __init__(self):
        self.weights = ROOT + 'models/best.pt'  # model path or triton URL
        self.data = ROOT + 'data/lscd_2cls.yaml'  # dataset.yaml path
        self.conf_thres = 0.45  # confidence threshold
        self.iou_thres = 0.45  # NMS IOU threshold
        self.max_det = 1000  # maximum detections per image
        self.device = ''  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        self.classes = None  # filter by class: --class 0, or --class 0 2 3
        self.agnostic_nms = False  # class-agnostic NMS
        self.line_thickness = 3  # bounding box thickness (pixels)
        self.hide_labels = False  # hide labels
        self.hide_conf = False  # hide confidences
        self.half = False  # use FP16 half-precision inference
        self.dnn = False  # use OpenCV DNN for ONNX inference
        self.vid_stride = 1  # video frame-rate stride

        self.imgsz = (640, 640)  # inference size (height, width)
        self.imgsz *= 2 if len(self.imgsz) == 1 else 1  # expand

        # Load model
        device = select_device(self.device)
        self.model = DetectMultiBackend(self.weights, device=device, dnn=self.dnn, data=self.data, fp16=self.half)
        stride, self.names, pt = self.model.stride, self.model.names, self.model.pt
        self.imgsz = check_img_size(self.imgsz, s=stride)  # check image size

        # Run inference
        self.model.warmup(imgsz=(1 if pt or self.model.triton else 1, 3, *self.imgsz))  # warmup

    @smart_inference_mode()
    def inference(self, im0):
        im = letterbox(im0, self.imgsz[0], stride=32, auto=True)[0]  # padded resize
        im = im.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        im = np.ascontiguousarray(im)  # contiguous

        # Pre-process
        im = torch.from_numpy(im).to(self.model.device)
        im = im.half() if self.model.fp16 else im.float()  # uint8 to fp16/32
        im /= 255  # 0 - 255 to 0.0 - 1.0
        if len(im.shape) == 3:
            im = im[None]  # expand for batch dim

        # Inference
        pred = self.model(im, augment=False, visualize=False)

        # NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms,
                                   max_det=self.max_det)

        # Process predictions
        annotator = Annotator(im0, line_width=self.line_thickness, example=str(self.names))
        det = pred[0]
        if len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()

            # Write results
            for *xyxy, conf, cls in reversed(det):
                c = int(cls)  # integer class
                label = None if self.hide_labels else (self.names[c] if self.hide_conf else f'{self.names[c]} {conf:.2f}')
                annotator.box_label(xyxy, label, color=colors(c, True))
        # Stream results
        im0 = annotator.result()

        return im0, det

# def main(opt):
#     check_requirements(exclude=('tensorboard', 'thop'))
#     run(**vars(opt))
#
#
# if __name__ == "__main__":
#     opt = parse_opt()
#     main(opt)
