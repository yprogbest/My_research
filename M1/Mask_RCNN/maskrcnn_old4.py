from mrcnn.config import Config
from mrcnn import model as modellib
from mrcnn import visualize
import numpy as np
import colorsys
import argparse
import imutils
import random
import cv2
import os
import cv2



def apply_mask(f, f_object_name, label, image, mask, color, alpha=0.5):
    """Apply the given mask to the image.
    """

    num_per_image = 0

    original_image = image.copy()
    y_len = original_image.shape[0]
    x_len = original_image.shape[1]


    for c in range(3):
        image[:, :, c] = np.where(mask == 1,
                                  image[:, :, c] *
                                  (1 - alpha) + alpha * color[c] * 255,
                                  image[:, :, c])

        mask_image = image
        mask_y_len = mask_image.shape[0]
        mask_x_len = mask_image.shape[1]

    
    f_object_name.write(label + "\n")

    # extract coodinates of different colors
    for j in range(y_len):
        for i in range(x_len):

            if original_image[j,i][0] != mask_image[j,i][0] and original_image[j,i][1] != mask_image[j,i][1] and original_image[j,i][2] != mask_image[j,i][2]:

                print(i,j)

                f.write(str(i) + "\t" + str(j))
                f.write("\n")

    f.write("-\n")

    f_object_name.write("-\n")


    #print(image.shape)


    return mask_image









def main():

    num = 0

    #write the coodinates that were extracted on text
    text_dir = "/home/ecb/instnce_segmentation/Mask_RCNN/result/text/"
    #text_dir = "/home/ecb/instnce_segmentation/Mask_RCNN/result/text2/"

    #write only object name
    text_object_name_dir = "/home/ecb/instnce_segmentation/Mask_RCNN/result/text_object_name/"
    #text_object_name_dir = "/home/ecb/instnce_segmentation/Mask_RCNN/result/text_object_name2/"



    #CLASS_NAMES =['BG', 'person', 'container']
    CLASS_NAMES =['BG', 'container', 'person']
    #CLASS_NAMES =['BG', 'person']

    # CLASS_NAMES =['BG', 'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 
    #              'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
    #              'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
    #              'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 
    #              'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite', 
    #              'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket', 
    #              'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 
    #              'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 
    #              'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'dining table', 
    #              'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 
    #              'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 
    #              'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush']


    hsv = [(i / len(CLASS_NAMES), 1, 1.0) for i in range(len(CLASS_NAMES))]
    COLORS = list(map(lambda c: colorsys.hsv_to_rgb(*c), hsv))
    random.seed(42)
    random.shuffle(COLORS)

    class SimpleConfig(Config):
        #NAME = "coco_inference"
        NAME = "farm"
        GPU_COUNT = 1
        IMAGES_PER_GPU = 1
        NUM_CLASSES = len(CLASS_NAMES)

    config = SimpleConfig()

    print("[INFO] loading Mask R-CNN model...")
    model = modellib.MaskRCNN(mode="inference", config=config,
        model_dir=os.getcwd())

    #学習データの指定
    model.load_weights("logs/farm20210605T2306/mask_rcnn_farm_0099.h5", by_name=True)
    #model.load_weights("mask_rcnn_coco.h5", by_name=True, xclude=["mrcnn_class_logits", "mrcnn_bbox_fc","mrcnn_bbox", "mrcnn_mask"])


    #自分が表示したい画像を指定する
    #image = cv2.imread("images/image369.png")
    #image = cv2.imread("images/02.jpg")

    #Use stereo camera
    input_folder = "/home/ecb/instnce_segmentation/Mask_RCNN/"


    input_folder = input_folder + "image_to_movie_20201117_Container_around_workshop_low_position"
    #input_folder = input_folder + "image_to_movie_20201223112457"


    input_image = input_folder + "/out_cam1_remap.mov"
    cap = cv2.VideoCapture(input_image)

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) # 動画の画面横幅
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) # 動画の画面縦幅
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) # 総フレーム数


    #size = (width, height)
    frame_rate = 5.0 # フレームレート
    fmt = cv2.VideoWriter_fourcc('m', 'p', '4', 'v') # ファイル形式(ここではmp4)
    save_folder = "/home/ecb/instnce_segmentation/Mask_RCNN/result/"
    save_movie=save_folder+"output.mov"
    video = cv2.VideoWriter(save_movie, fmt, frame_rate, (width,height)) # ライター作成






    while True:
        #open file
        text  = text_dir + "result" + str(num) + ".txt"
        f = open(text, "w")


        #open file
        text_object_name  = text_object_name_dir + "result_object_name" + str(num) + ".txt"
        f_object_name = open(text_object_name, "w")



        ret, image = cap.read()

        if image is None:
            break

        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # image = imutils.resize(image, width=512)



        print("[INFO] making predictions with Mask R-CNN...")
        #学習データに画像を処理させる
        r = model.detect([image], verbose=1)[0]
        #rに処理データが格納される
        '''
        ["rois"]：範囲の4点が格納されている
        ["class_ids"]：クラスID、0から80の数が入っている。画像に何が写っているかを教えてくれる。
        ["masks"]：ピクセル毎の範囲指定
        '''

        N = r['rois'].shape[0]

        for i in range(0, N):
            classID = r["class_ids"][i]
            label = CLASS_NAMES[classID]
            mask = r["masks"][:, :, i]
            color = COLORS[classID][::-1]

            image = apply_mask(f, f_object_name, label, image, mask, color, alpha=0.5)

            # image = visualize.display_instances(image, r['rois'], r['masks'], r['class_ids'], 
            #                     CLASS_NAMES, r['scores'])


        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        for i in range(0, len(r["scores"])):
            (startY, startX, endY, endX) = r["rois"][i]
            classID = r["class_ids"][i]
            label = CLASS_NAMES[classID]
            score = r["scores"][i]
            color = [int(c) for c in np.array(COLORS[classID]) * 255]

            cv2.rectangle(image, (startX, startY), (endX, endY), color, 2)
            text = "{}: {:.3f}".format(label, score)
            y = startY - 10 if startY - 10 > 10 else startY + 10
            cv2.putText(image, text, (startX, y), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, color, 2)


        
        num = num + 1

        f.close()
        f_object_name.close()
    

        cv2.imshow("Output", image)
        #print(image.shape)

        #save
        video.write(image)
        #cv2.waitKey()
        
        key = cv2.waitKey(30)
        if key>=0:
            break


    cap.release()
    video.release()
    cv2.destroyAllWindows()




if __name__ == "__main__":
    
    main()
