import time

import rclpy
import torch.cuda
from rclpy.action import ActionServer
from rclpy.node import Node

from franka_action_interface.action import ImageSeg
from U2_Net.u2net_run import *


class ImageSegServer(Node):
    def __init__(self):
        super().__init__('reach_action_server')

        # nn model initialization
        abs_path = os.path.dirname(os.path.abspath(__file__))
        model_name = 'u2net'
        model_dir = os.path.join(abs_path, "U2_Net", 'saved_models', model_name, model_name + '.pth')
        self.net = U2NET(3, 1)
        if torch.cuda.is_available():
            self.net.load_state_dict(torch.load(model_dir))
            self.net.cuda()
        else:
            self.net.load_state_dict(torch.load(model_dir, map_location='cpu'))
        self.net.eval()

        # ROS2 Action server initialization
        self._action_server = ActionServer(
            self,
            ImageSeg,
            'ImageSeg',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # Load path
        target_path = goal_handle.request.image_dir_path
        self.get_logger().info('The target path is: {}'.format(target_path))
        image_dir = os.path.join(target_path, 'input')
        prediction_dir = os.path.join(target_path, 'temp' + os.sep)
        segmentation_dir = os.path.join(target_path, 'output' + os.sep)
        img_name_list = glob.glob(image_dir + os.sep + '*')

        # Load data
        test_salobj_dataset = SalObjDataset(img_name_list=img_name_list,
                                            lbl_name_list=[],
                                            transform=transforms.Compose([RescaleT(320),
                                                                          ToTensorLab(flag=0)])
                                            )
        test_salobj_dataloader = DataLoader(test_salobj_dataset,
                                            batch_size=1,
                                            shuffle=False,
                                            num_workers=1)

        # Processing
        for i_test, data_test in enumerate(test_salobj_dataloader):

            print("inferencing:", img_name_list[i_test].split(os.sep)[-1])

            inputs_test = data_test['image']
            inputs_test = inputs_test.type(torch.FloatTensor)

            if torch.cuda.is_available():
                inputs_test = Variable(inputs_test.cuda())
            else:
                inputs_test = Variable(inputs_test)

            d1, d2, d3, d4, d5, d6, d7 = self.net(inputs_test)

            # normalization
            d1 = d1[:, 0, :, :]
            d1 = normPRED(d1)

            # save results to test_results folder
            if not os.path.exists(prediction_dir):
                os.makedirs(prediction_dir, exist_ok=True)
            save_output_overlay(img_name_list[i_test], d1, prediction_dir, segmentation_dir)

            del d1, d2, d3, d4, d5, d6, d7
            torch.cuda.empty_cache()

            time.sleep(0.1)

        result = ImageSeg.Result()
        result.success_flag = True
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)

    reach_action_server = ImageSegServer()

    rclpy.spin(reach_action_server)


if __name__ == '__main__':
    main()
