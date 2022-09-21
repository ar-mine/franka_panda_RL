import os
from PIL import Image
from importlib.machinery import SourceFileLoader
import torch
from tqdm import tqdm
import numpy as np
import pandas
import numpy
import cv2
from math import atan2, atan

from .siamesenet import SiameseNetwork
from torchvision import transforms
import torch.nn.functional as F


class DTOIDModule:
    def __init__(self, template_dir, backend="cuda", no_filter_z=True):
        self.no_filter_z = no_filter_z
        self.backend = backend

        # Template paths
        file_path = os.path.dirname(os.path.abspath(__file__))
        model_directory = os.path.join(file_path, "templates")
        self.template_dir = os.path.join(model_directory, template_dir)

        # Load Network
        network_module = SourceFileLoader(file_path, os.path.join(file_path, "network.py")).load_module()
        self.model = network_module.Network()
        self.model.eval()
        checkpoint = torch.load(os.path.join(file_path, "model.pth.tar"), map_location=lambda storage, loc: storage)

        self.model.load_state_dict(checkpoint["state_dict"])
        if self.backend == "cuda":
            self.model = self.model.cuda()
        self.preprocess = network_module.PREPROCESS

        # Load Siamese Network
        self.sim_model = SiameseNetwork()
        self.sim_model.eval()
        self.sim_model = self.sim_model.cuda()
        simnet_checkpoint = torch.load(os.path.join(file_path, "model_15.pth.tar"))
        try:
            self.sim_model.module.load_state_dict(simnet_checkpoint, strict=True)
        except:
            self.sim_model.load_state_dict(simnet_checkpoint, strict=True)

        normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        self.transformTemp = transforms.Compose([
            transforms.Resize((64, 64)),
            transforms.ToTensor(),
            normalize,
        ])

        # load pose file
        if not self.no_filter_z:
            pose_file = os.path.join(self.template_dir, "poses.txt")
            pose_file_np = pandas.read_csv(pose_file, sep=" ", header=None).values
            self.pose_z_values = pose_file_np[:, 11]

        # Template
        self.template_paths = [x for x in os.listdir(self.template_dir)]
        self.template_paths.sort()

        # features for all templates (240)
        self.template_list = []
        self.template_global_list = []

        batch_size = 10
        temp_batch_local = []
        iteration = 0

        # Load template
        for t in tqdm(self.template_paths):
            # open template and template mask
            template_im = cv2.imread(os.path.join(self.template_dir, t))[:, :, ::-1]
            template = Image.fromarray(template_im)

            template_mask = cv2.imread(os.path.join(self.template_dir, t.replace("_a", "_m")))[:, :, 0]
            template_mask = Image.fromarray(template_mask)

            # preprocess and concatenate
            template = self.preprocess[1](template)
            template_mask = self.preprocess[2](template_mask)
            template = torch.cat([template, template_mask], dim=0)

            if self.backend == "cuda":
                template = template.cuda()

            template_feature = self.model.compute_template_local(template.unsqueeze(0))

            # Create mini-batches of templates
            if iteration == 0:
                temp_batch_local = template_feature

                template_feature_global = self.model.compute_template_global(template.unsqueeze(0))
                self.template_global_list.append(template_feature_global)

            elif iteration % batch_size == 0:
                self.template_list.append(temp_batch_local)
                temp_batch_local = template_feature

            elif iteration == (len(self.template_paths) - 1):
                temp_batch_local = torch.cat([temp_batch_local, template_feature], dim=0)
                self.template_list.append(temp_batch_local)

            else:
                temp_batch_local = torch.cat([temp_batch_local, template_feature], dim=0)

            iteration += 1

    def process(self, rgb_img: np.ndarray, threshold: float = 0.7) -> (bool, np.ndarray, tuple, float):
        """
        Detect target instance object from @rgb_img while filtering whose scores are under @threshold.
        :param rgb_img: omit
        :param threshold: omit
        :return: successful flag(whether detecting object with scores higher than @threshold)
                 @rgb_image with overlay bounding boxes
                 coordinates of bounding box(so far without rotation)
                 angle of the rotation
        """
        # Copy input to avoid disturb public var.
        img_copy = rgb_img.copy()
        img_h, img_w, img_c = img_copy.shape

        img_tensor = Image.fromarray(img_copy[:, :, ::-1])
        img_tensor = self.preprocess[0](img_tensor)

        network_h = img_tensor.size(1)
        network_w = img_tensor.size(2)
        if self.backend == "cuda":
            img_tensor = img_tensor.cuda()

        top_k_num = 500
        top_k_scores, top_k_bboxes, top_k_template_ids = self.model.forward_all_templates(
            img_tensor.unsqueeze(0), self.template_list, self.template_global_list, topk=top_k_num)

        pred_scores_np = top_k_scores.cpu().numpy()
        pred_bbox_np = top_k_bboxes.cpu().numpy()
        pred_template_ids = top_k_template_ids[:, 0].long().cpu().numpy()

        if not self.no_filter_z:
            template_z_values = self.pose_z_values[pred_template_ids]

            pred_w_np = pred_bbox_np[:, 2] - pred_bbox_np[:, 0]
            pred_h_np = pred_bbox_np[:, 3] - pred_bbox_np[:, 1]
            pred_max_dim_np = np.stack([pred_w_np, pred_h_np]).transpose().max(axis=1)
            pred_z = (124 / pred_max_dim_np) * -template_z_values

            # Filter based on predicted Z values
            pred_z_conds = (pred_z > 0.4) & (pred_z < 2)
            pred_z_conds_ids = numpy.where(pred_z_conds)[0]

            pred_scores_np = pred_scores_np[pred_z_conds_ids]
            pred_bbox_np = pred_bbox_np[pred_z_conds_ids]
            pred_template_ids = pred_template_ids[pred_z_conds_ids]
            pred_z = pred_z[pred_z_conds_ids]

        #########################################
        # leveraging siamese net to filtering the proposed bbox
        temp_img = Image.open(os.path.join(self.template_dir, self.template_paths[10])).convert('RGB')
        # temp_img.save('./prediction/%s_temp.jpg'%(data['img_path'].split('/')[-1].split('.')[0]))
        temp_img = self.transformTemp(temp_img)
        temp_img = temp_img.cuda()
        temp_img = torch.unsqueeze(temp_img, 0)

        disimilar_list = []
        seg_list = []

        for p in range(pred_scores_np.shape[0]):
            x1, y1, x2, y2 = pred_bbox_np[p]
            # temp_score = pred_scores_np[p]
            x1 = int(x1 / network_w * img_w)
            x2 = int(x2 / network_w * img_w)
            y1 = int(y1 / network_h * img_h)
            y2 = int(y2 / network_h * img_h)

            bbox_img = img_copy[y1:y2, x1:x2, :]
            # bbox_img=np.moveaxis(bbox_img,-1,0)
            # cv2.imwrite('./prediction/%s_%02d.jpg'%(data['img_path'].split('/')[-1].split('.')[0],p),bbox_img)

            bbox_img = Image.fromarray(bbox_img[:, :, ::-1])
            bbox_img = self.transformTemp(bbox_img)
            bbox_img = bbox_img.cuda()
            bbox_img = torch.unsqueeze(bbox_img, 0)

            output1, output2, seg_img = self.sim_model(temp_img, bbox_img, True)
            euclidean_distance = F.pairwise_distance(output1, output2)

            euclidean_distance = euclidean_distance.cpu().detach().numpy()

            disimilar_list.append(euclidean_distance[0])

            seg_img = seg_img.cpu().detach().numpy()
            seg_list.append(seg_img)

        disimilar_list = np.array(disimilar_list)
        disimilar_list = disimilar_list / np.sum(disimilar_list)
        # print(pred_scores_np,disimilar_list)

        # update the sort of prediction
        new_pred_scores_np = pred_scores_np - disimilar_list
        # print(pred_scores_np,new_pred_scores_np)
        sort_indices = np.argsort(new_pred_scores_np)[::-1]

        pred_scores_np = pred_scores_np[sort_indices]
        pred_bbox_np = pred_bbox_np[sort_indices]

        seg_list = np.array(seg_list)
        seg_list = seg_list[sort_indices]
        # print(seg_list.shape)
        #########################################

        # Keep top 1 (eval)
        pred_scores_np = pred_scores_np[:1]
        pred_bbox_np = pred_bbox_np[:1]
        pred_template_ids = pred_template_ids[:1]
        if not self.no_filter_z:
            pred_z = pred_z[:1]
        # print("pred_scores_np: %f" % pred_scores_np)
        # Show prediction
        success = False
        bbox = (0, 0, 0, 0)
        angel = 0

        if len(pred_bbox_np) > 0 and pred_scores_np >= threshold:
            x1, y1, x2, y2 = pred_bbox_np[0]
            temp_score = pred_scores_np[0]

            x1 = int(x1 / network_w * img_w)
            x2 = int(x2 / network_w * img_w)
            y1 = int(y1 / network_h * img_h)
            y2 = int(y2 / network_h * img_h)

            rec_color = (0, 255, 255)
            cv2.rectangle(img_copy,
                          (x1, y1),
                          (x2, y2),
                          rec_color, 3)
            ################################
            # get the object mask
            seg_img = seg_list[0].squeeze()
            seg_img = sigmoid(seg_img) * 255
            seg_img = seg_img.astype(np.uint8)

            seg_img = cv2.resize(seg_img, (x2 - x1, y2 - y1))

            _, RedThresh = cv2.threshold(seg_img, 50, 255, cv2.THRESH_BINARY)

            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            # eroded = cv2.erode(RedThresh,kernel)
            dilated = cv2.dilate(RedThresh, kernel)

            '''
            #add mask
            img_mask = np.zeros((seg_img.shape[0],seg_img.shape[1],3),np.uint8)
            img_mask[RedThresh>0]=[0,0,255]
            
            img_mask = cv2.addWeighted(img_mask,0.5,img_numpy[y1:y2,x1:x2],0.5,0)
            #cv2.imwrite('./test_img/test_video/%05d_mask.jpg'%(frame_idx),img_mask)
            img_numpy[y1:y2,x1:x2] = img_mask
            '''

            # find contours
            contours = cv2.findContours(RedThresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
            # cv2.imwrite('./test_img/test_video/%05d_mask.jpg'%(frame_idx),RedThresh)

            wh = 0

            for contour in contours:

                approx = cv2.approxPolyDP(
                    contour, 0.01 * cv2.arcLength(contour, True), True)

                x, y, w, h = cv2.boundingRect(contour)
                if w < 10 or h < 10 or w > 300 or h > 300:
                    continue
                if w * h > wh:
                    wh = w * h
                    best_contour = contour

            rect = cv2.minAreaRect(best_contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)

            box[:, 0] = box[:, 0] + x1
            box[:, 1] = box[:, 1] + y1

            # using drawContours() function
            cv2.drawContours(img_copy, [box], 0, (0, 0, 255), 2)

            vect = [box[1, :] - box[0, :], box[2, :] - box[1, :], box[3, :] - box[2, :], box[0, :] - box[3, :]]
            vect_norm = np.linalg.norm(vect, axis=1)
            vec = vect[np.argmax(vect_norm)]
            rot_z = atan(vec[1] / vec[0]) - np.pi
            #################################

            success = True
            bbox = (x1, y1, x2, y2)
            # bbox = (int(rect[0][0]), int(rect[0][1]), int(rect[1][0]), int(rect[1][1]))
            angel = rot_z

        return success, img_copy, bbox, angel


def sigmoid(x):
    s = 1 / (1 + np.exp(-x))
    return s


if __name__ == "__main__":
    image_path = "/home/armine/Pictures/YCB-like template dataset/scenarios/clutter env/sequence"
    template_path = "/home/armine/Pictures/YCB-like template dataset/templates/car/output"
    runner = DTOIDModule(template_dir=template_path)
    img_list = os.listdir(image_path)
    for img_name in img_list:
        img = cv2.imread(os.path.join(image_path, img_name))
        success, img_numpy, bbox, angel = runner.process(img)
        cv2.imshow("show", img_numpy)
        cv2.waitKey(1)
