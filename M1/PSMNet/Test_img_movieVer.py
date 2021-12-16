from __future__ import print_function
import argparse
import os
import random
import torch
import torch.nn as nn
import torchvision.transforms as transforms
import torch.nn.functional as F
import numpy as np
import time
import math
from models import *
import cv2
from PIL import Image
import numpy as np

# 2012 data /media/jiaren/ImageNet/data_scene_flow_2012/testing/

parser = argparse.ArgumentParser(description='PSMNet')
parser.add_argument('--KITTI', default='2015',
                    help='KITTI version')
parser.add_argument('--datapath', default='/media/jiaren/ImageNet/data_scene_flow_2015/testing/',
                    help='select model')
parser.add_argument('--loadmodel', default='./trained/pretrained_model_KITTI2015.tar',
                    help='loading model')
parser.add_argument('--leftimg', default= './movie_stereo_cam1.mov',
                    help='load model')
parser.add_argument('--rightimg', default= './movie_stereo_cam2.mov',
                    help='load model')                                      
parser.add_argument('--model', default='stackhourglass',
                    help='select model')
parser.add_argument('--maxdisp', type=int, default=192,
                    help='maxium disparity')
parser.add_argument('--no-cuda', action='store_true', default=False,
                    help='enables CUDA training')
parser.add_argument('--seed', type=int, default=1, metavar='S',
                    help='random seed (default: 1)')
args = parser.parse_args()
args.cuda = not args.no_cuda and torch.cuda.is_available()

torch.manual_seed(args.seed)
if args.cuda:
    torch.cuda.manual_seed(args.seed)

if args.model == 'stackhourglass':
    model = stackhourglass(args.maxdisp)
elif args.model == 'basic':
    model = basic(args.maxdisp)
else:
    print('no model')

model = nn.DataParallel(model, device_ids=[0])
model.cuda()

if args.loadmodel is not None:
    print('load PSMNet')
    state_dict = torch.load(args.loadmodel)
    model.load_state_dict(state_dict['state_dict'])

print('Number of model parameters: {}'.format(sum([p.data.nelement() for p in model.parameters()])))

def test(imgL,imgR):
        model.eval()

        if args.cuda:
           imgL = imgL.cuda()
           imgR = imgR.cuda()     

        with torch.no_grad():
            disp = model(imgL,imgR)

        disp = torch.squeeze(disp)
        pred_disp = disp.data.cpu().numpy()

        

        return pred_disp



def pil_to_cv2(pil_image):
    'PIL -> CV2'

    # pil_imageをNumPy配列に変換
    pil_image_array = np.array(pil_image)

    # RGB -> BGR によりCV2画像オブジェクトに変換
    cv2_image = cv2.cvtColor(pil_image_array, cv2.COLOR_RGB2BGR)

    return cv2_image



def main():

        frame_count = 0

        normal_mean_var = {'mean': [0.485, 0.456, 0.406],
                            'std': [0.229, 0.224, 0.225]}
        infer_transform = transforms.Compose([transforms.ToTensor(),
                                              transforms.Normalize(**normal_mean_var)])    

                            
        cap_left = cv2.VideoCapture(args.leftimg)
        cap_left_width = cap_left.get(cv2.CAP_PROP_FRAME_WIDTH)
        cap_left_height = cap_left.get(cv2.CAP_PROP_FRAME_HEIGHT)
        cap_size = (cap_left_width, cap_left_height)

        cap_right = cv2.VideoCapture(args.rightimg)

        output_movie_name = "./image_to_movie/stereo_PSMNET.mov"
        output_movie = cv2.VideoWriter(output_movie_name, cv2.VideoWriter_fourcc(*'MP4V'), 30.0, (1280, 720))



        while(cap_left.isOpened() and cap_right.isOpened()):

            ret_left, imgL_o = cap_left.read()
            # imgL_o = Image.open(imgL_o).convert('RGB')
            imgL_o = cv2.cvtColor(imgL_o, cv2.COLOR_BGR2RGB)

            ret_right, imgR_o = cap_right.read()
            # imgR_o = Image.open(imgR_o).convert('RGB')
            imgR_o = cv2.cvtColor(imgR_o, cv2.COLOR_BGR2RGB)


            imgL = infer_transform(imgL_o)
            imgR = infer_transform(imgR_o) 
       

            # pad to width and hight to 16 times
            if imgL.shape[1] % 16 != 0:
                times = imgL.shape[1]//16       
                top_pad = (times+1)*16 -imgL.shape[1]
            else:
                top_pad = 0

            if imgL.shape[2] % 16 != 0:
                times = imgL.shape[2]//16                       
                right_pad = (times+1)*16-imgL.shape[2]
            else:
                right_pad = 0    

            imgL = F.pad(imgL,(0,right_pad, top_pad,0)).unsqueeze(0)
            imgR = F.pad(imgR,(0,right_pad, top_pad,0)).unsqueeze(0)


            start_time = time.time()



            pred_disp = test(imgL,imgR)
            print('time = %.2f' %(time.time() - start_time))

            
            if top_pad !=0 and right_pad != 0:
                img = pred_disp[top_pad:,:-right_pad]
            elif top_pad ==0 and right_pad != 0:
                img = pred_disp[:,:-right_pad]
            elif top_pad !=0 and right_pad == 0:
                img = pred_disp[top_pad:,:]
            else:
                img = pred_disp
            
            img = (img*256).astype('uint8')
            img = Image.fromarray(img)

            #img = pil_to_cv2(img)
            #output_movie.write(img)

            img = np.array(img)
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            output_movie.write(img)


            output_file = open('output_pred_disp/output_pred_disp' + str(frame_count) + '.txt', 'w')

            h,w = pred_disp.shape

            for j in range(0, h-1):
                for i in range(0, w-1):
                    output_file.write(str(pred_disp[j,i])+"\t")
                output_file.write("\n")


            output_file.close

            print(pred_disp)


            frame_count = frame_count + 1


        output_movie.release()


if __name__ == '__main__':
   main()






