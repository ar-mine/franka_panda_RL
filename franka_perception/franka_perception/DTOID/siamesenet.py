import json

import cv2
import math
import torch
import torchvision

import numpy as np
import random
from torch import nn
import torch.nn.functional as F
from torch.autograd import Variable

from torchvision import models
from torchvision.transforms import transforms


#from lib.nms.pth_nms import pth_nms
# from lib.retinanet.Resnets import resnet18 as LibResNet


class DoubleConv(nn.Module):
    """(convolution => [BN] => ReLU) * 2"""

    def __init__(self, in_channels, out_channels, mid_channels=None):
        super().__init__()
        if not mid_channels:
            mid_channels = out_channels
        self.double_conv = nn.Sequential(
            nn.Conv2d(in_channels, mid_channels, kernel_size=3, padding=1, bias=False),
            nn.BatchNorm2d(mid_channels),
            nn.ReLU(inplace=True),
            nn.Conv2d(mid_channels, out_channels, kernel_size=3, padding=1, bias=False),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True)
        )

    def forward(self, x):
        return self.double_conv(x)

class Down(nn.Module):
    """Downscaling with maxpool then double conv"""

    def __init__(self, in_channels, out_channels):
        super().__init__()
        self.maxpool_conv = nn.Sequential(
            nn.MaxPool2d(2),
            DoubleConv(in_channels, out_channels)
        )

    def forward(self, x):
        return self.maxpool_conv(x)


class Up(nn.Module):
    """Upscaling then double conv"""

    def __init__(self, in_channels, out_channels, bilinear=True):
        super().__init__()

        # if bilinear, use the normal convolutions to reduce the number of channels
        if bilinear:
            self.up = nn.Upsample(scale_factor=2, mode='bilinear', align_corners=True)
            self.conv = DoubleConv(in_channels, out_channels, in_channels // 2)
        else:
            self.up = nn.ConvTranspose2d(in_channels, in_channels // 2, kernel_size=2, stride=2)
            self.conv = DoubleConv(in_channels, out_channels)

    def forward(self, x1, x2):
        x1 = self.up(x1)
        # input is CHW
        diffY = x2.size()[2] - x1.size()[2]
        diffX = x2.size()[3] - x1.size()[3]

        x1 = F.pad(x1, [diffX // 2, diffX - diffX // 2,
                        diffY // 2, diffY - diffY // 2])
        # if you have padding issues, see
        # https://github.com/HaiyongJiang/U-Net-Pytorch-Unstructured-Buggy/commit/0e854509c2cea854e247a9c615f175f76fbb2e3a
        # https://github.com/xiaopeng-liao/Pytorch-UNet/commit/8ebac70e633bac59fc22bb5195e513d5832fb3bd
        x = torch.cat([x2, x1], dim=1)
        return self.conv(x)


class OutConv(nn.Module):
    def __init__(self, in_channels, out_channels):
        super(OutConv, self).__init__()
        self.conv = nn.Conv2d(in_channels, out_channels, kernel_size=1)

    def forward(self, x):
        return self.conv(x)

FEATURE_NUM=64

class SiameseNetwork(nn.Module):
      def __init__(self):
            super(SiameseNetwork, self).__init__()
            '''
            self.cnn1 = nn.Sequential(
                  #nn.ReflectionPad2d(1),
                  nn.Conv2d(3, 48, kernel_size=3, stride=2, padding=1),
                  nn.ReLU(inplace=True),
                  nn.BatchNorm2d(48),
                  #nn.Dropout2d(p=.2),
                  
                  #nn.ReflectionPad2d(1),
                  nn.Conv2d(48, 128, kernel_size=3, stride=2, padding=1),
                  nn.ReLU(inplace=True),
                  nn.BatchNorm2d(128),
                  #nn.Dropout2d(p=.2),
                      
                  #nn.ReflectionPad2d(1),
                  nn.Conv2d(128, 256, kernel_size=3, stride=2, padding=1),
                  nn.ReLU(inplace=True),
                  nn.BatchNorm2d(256),
                  #nn.Dropout2d(p=.2),
                  
                  nn.MaxPool2d(8)
                  )
            
            self.cnn1 = LibResNet(pretrained=True)
            
            self.fc1 = nn.Sequential(
                  nn.Linear(512, 512),
                  nn.ReLU(inplace=True),
                  
                  nn.Linear(512, 256),
                  nn.ReLU(inplace=True),
                  
                  nn.Linear(256, 128)
                  )
            
            self.fc1 = nn.Linear(2*512, 512)
            #self.fc1 = nn.Linear(2*1000, 500)
            self.fc2 = nn.Linear(512, 512)
            self.fc3 = nn.Linear(512, 2)
            '''
            
            self.n_channels = 3
            self.n_classes = 1
            self.bilinear = True
    
            self.inc = DoubleConv(self.n_channels, FEATURE_NUM)
            self.down1 = Down(FEATURE_NUM, FEATURE_NUM*2)
            self.down2 = Down(FEATURE_NUM*2, FEATURE_NUM*4)
            self.down3 = Down(FEATURE_NUM*4, FEATURE_NUM*8)
            factor = 2 if self.bilinear else 1
            self.down4 = Down(FEATURE_NUM*8, FEATURE_NUM*16 // factor)
            self.up1 = Up(FEATURE_NUM*16, FEATURE_NUM*8 // factor, self.bilinear)
            self.up2 = Up(FEATURE_NUM*8, FEATURE_NUM*4 // factor, self.bilinear)
            self.up3 = Up(FEATURE_NUM*4, FEATURE_NUM*2 // factor, self.bilinear)
            self.up4 = Up(FEATURE_NUM*2, FEATURE_NUM, self.bilinear)
            self.outc = OutConv(FEATURE_NUM, self.n_classes)
            
            self.fc1 = nn.Sequential(
                  nn.Linear(FEATURE_NUM*8, FEATURE_NUM*8),
                  nn.ReLU(inplace=True),
                  
                  nn.Linear(FEATURE_NUM*8, FEATURE_NUM*4),
                  nn.ReLU(inplace=True),
                  
                  nn.Linear(FEATURE_NUM*4, 64)
                  )
            self.avgpool = nn.AdaptiveAvgPool2d((1, 1))

      def forward_once(self, x):
            '''
            output = self.cnn1(x)
            #print(output.shape)
            output = output.view(output.size()[0], -1)
            output = self.fc1(output)
            return output
            '''
            x1 = self.inc(x)
            x2 = self.down1(x1)
            x3 = self.down2(x2)
            x4 = self.down3(x3)
            x5 = self.down4(x4)
            
            output = self.avgpool(x5)
            output = torch.flatten(output, 1)
            output = output.view(output.size()[0], -1)
            output = self.fc1(output)
            return output, x5, x4, x3, x2, x1
      
      
      def forward(self, input1, input2, is_seg=True):
             output1, _ , _ , _ , _ , _  = self.forward_once(input1)
             output2, x5, x4, x3, x2, x1 = self.forward_once(input2)
             
             if not is_seg:
                return output1, output2, _
                
             x = self.up1(x5, x4)
             x = self.up2(x, x3)
             x = self.up3(x, x2)
             x = self.up4(x, x1)
             logits = self.outc(x)
             
             
             '''
             output1 = self.cnn1(input1)
             output1 = output1.view(output1.size()[0], -1)#make it suitable for fc layer.
             output2 = self.cnn1(input2)
             output2 = output2.view(output2.size()[0], -1)
              
             output = torch.cat((output1, output2),1)
             output = F.relu(self.fc1(output))
             output = F.relu(self.fc2(output))
             output = self.fc3(output)
             return output
             '''
             
             return output1, output2, logits