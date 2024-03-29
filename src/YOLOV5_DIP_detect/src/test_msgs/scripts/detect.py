import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

import sys
print(sys.path)
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

import argparse
import math
import time
from pathlib import Path

import cv2
import torch
import torch.backends.cudnn as cudnn
from numpy import random

from models.experimental import attempt_load
from utils.datasets import LoadStreams, LoadImages
from utils.general import check_img_size, check_requirements, check_imshow, non_max_suppression, apply_classifier, \
    scale_coords, xyxy2xywh, strip_optimizer, set_logging, increment_path
from utils.plots import plot_one_box
from utils.torch_utils import select_device, load_classifier, time_synchronized


def detect(save_img=False):
    source, weights, view_img, save_txt, imgsz = opt.source, opt.weights, opt.view_img, opt.save_txt, opt.img_size
    save_img = not opt.nosave and not source.endswith('.txt')  # save inference images
    webcam = source.isnumeric() or source.endswith('.txt') or source.lower().startswith(
        ('rtsp://', 'rtmp://', 'http://', 'https://'))

    # Directories
    save_dir = Path(increment_path(Path(opt.project) / opt.name, exist_ok=opt.exist_ok))  # increment run
    (save_dir / 'labels' if save_txt else save_dir).mkdir(parents=True, exist_ok=True)  # make dir

    # Initialize
    set_logging()
    device = select_device(opt.device)
    half = device.type != 'cpu'  # half precision only supported on CUDA

    # Load model
    model = attempt_load(weights, map_location=device)  # load FP32 model
    stride = int(model.stride.max())  # model stride
    imgsz = check_img_size(imgsz, s=stride)  # check img_size
    if half:
        model.half()  # to FP16

    # Second-stage classifier
    classify = False
    if classify:
        modelc = load_classifier(name='resnet101', n=2)  # initialize
        modelc.load_state_dict(torch.load('weights/resnet101.pt', map_location=device)['model']).to(device).eval()

    # Set Dataloader
    vid_path, vid_writer = None, None
    if webcam:
        view_img = check_imshow()
        cudnn.benchmark = True  # set True to speed up constant image size inference
        dataset = LoadStreams(source, img_size=imgsz, stride=stride)
    else:
        dataset = LoadImages(source, img_size=imgsz, stride=stride)

    # Get names and colors
    names = model.module.names if hasattr(model, 'module') else model.names
    colors = [[random.randint(0, 255) for _ in range(3)] for _ in names]

    # Run inference
    if device.type != 'cpu':
        model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))  # run once
    t0 = time.time()
    for path, img, im0s, vid_cap in dataset:

        # result = []

        img = torch.from_numpy(img).to(device)
        img = img.half() if half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Inference
        t1 = time_synchronized()
        pred = model(img, augment=opt.augment)[0]

        # Apply NMS
        pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres, classes=opt.classes, agnostic=opt.agnostic_nms)
        t2 = time_synchronized()

        # Apply Classifier
        if classify:
            pred = apply_classifier(pred, modelc, img, im0s)

        # Process detections

        one_frame_result = []

        for i, det in enumerate(pred):  # detections per image
            if webcam:  # batch_size >= 1
                p, s, im0, frame = path[i], '%g: ' % i, im0s[i].copy(), dataset.count
            else:
                p, s, im0, frame = path, '', im0s, getattr(dataset, 'frame', 0)

            p = Path(p)  # to Path
            save_path = str(save_dir / p.name)  # img.jpg
            txt_path = str(save_dir / 'labels' / p.stem) + ('' if dataset.mode == 'image' else f'_{frame}')  # img.txt
            s += '%gx%g ' % img.shape[2:]  # print string
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

                # Write results
                # one_frame_result = []
                for *xyxy, conf, cls in reversed(det):
                    if save_txt:  # Write to file
                        xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                        line = (cls, *xywh, conf) if opt.save_conf else (cls, *xywh)  # label format
                        with open(txt_path + '.txt', 'a') as f:
                            f.write(('%g ' * len(line)).rstrip() % line + '\n')

                    if save_img or view_img:  # Add bbox to image
                        label = f'{names[int(cls)]} {conf:.2f}'
                        plot_one_box(xyxy, im0, label=label, color=colors[int(cls)], line_thickness=3)
                        x1 = int(xyxy[0].item())
                        y1 = int(xyxy[1].item())
                        x2 = int(xyxy[2].item())
                        y2 = int(xyxy[3].item())
                        sxy = abs((x1 - x2) * (y1 - y2))
                        # TODO
                        x_bar = (x1 + x2) / 2
                        y_bar = (y1 + y2) / 2
                        print(label + "\t x: {},\t y: {}".format(x_bar, y_bar))
                        if x_bar <= 640:
                            one_frame_result.append([names[int(cls)], int(cls), x_bar, y_bar, float(conf), sxy])

                for each_item in one_frame_result:
                    cv2.circle(im0, (int(each_item[2]), int(each_item[3])), 5, (0, 0, 0), -1)
                    cv2.putText(im0, str(each_item[0]) + " x:" + str(each_item[2]) + " y:" + str(each_item[3])+ " s:" + str(each_item[5]),
                                (int(each_item[2]), int(each_item[3])),
                                fontFace=cv2.FONT_HERSHEY_COMPLEX, fontScale=0.7, color=(0, 255, 0),
                                thickness=2)

            # Print time (inference + NMS)
            print(f'{s}Done. ({t2 - t1:.3f}s)')

            # Stream results
            if view_img:
                cv2.imshow(str(p), im0)
                cv2.waitKey(1)  # 1 millisecond

            # Save results (image with detections)
            if save_img:
                if dataset.mode == 'image':
                    cv2.imwrite(save_path, im0)
                else:  # 'video' or 'stream'
                    if vid_path != save_path:  # new video
                        vid_path = save_path
                        if isinstance(vid_writer, cv2.VideoWriter):
                            vid_writer.release()  # release previous video writer
                        if vid_cap:  # video
                            fps = vid_cap.get(cv2.CAP_PROP_FPS)
                            w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                            h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        else:  # stream
                            fps, w, h = 30, im0.shape[1], im0.shape[0]
                            save_path += '.mp4'
                        vid_writer = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
                    vid_writer.write(im0)

        num_red = 0
        # sumx = 0
        # sumy = 0
        # sums = 0
        redx=[]
        redy=[]
        reds=[]
        for each_item in one_frame_result:
            cv2.circle(im0, (int(each_item[2]), int(each_item[3])), 5, (0, 0, 0), -1)
            cv2.putText(im0, str(each_item[0]) + " x:" + str(each_item[2]) + " y:" + str(each_item[3])+ " s:" + str(each_item[5]),
                        (int(each_item[2]), int(each_item[3])),
                        fontFace=cv2.FONT_HERSHEY_COMPLEX, fontScale=0.7, color=(0, 255, 0),
                        thickness=2)
            if each_item[0] == 'red':
                redx.append(each_item[2])
                redy.append(each_item[3])
                reds.append(each_item[5])
                num_red = num_red + 1
                # sumx = each_item[2] + sumx
                # sumy = each_item[3] + sumy
                # sums = each_item[5] + sums

        red_msg1 = [-1, -1, -1]
        red_msg2 = [-1, -1, -1]
        # if num_red == 2:
        #     red_msg[0] = sumx / num_red
        #     red_msg[1] = sumy / num_red
        #     red_msg[2] = sums / num_red

        if num_red == 2:
            if  redx[0]<redx[1]:
                red_msg1[0] = redx[0]
                red_msg1[1] = redy[0]
                red_msg1[2] = reds[0]
                red_msg2[0] = redx[1]
                red_msg2[1] = redy[1]
                red_msg2[2] = reds[1]
            else:
                red_msg2[0] = redx[0]
                red_msg2[1] = redy[0]
                red_msg2[2] = reds[0]
                red_msg1[0] = redx[1]
                red_msg1[1] = redy[1]
                red_msg1[2] = reds[1]      

        blue_msg = [-1, -1, -1]
        nurse_msg = [-1, -1, -1]
        for each_item in one_frame_result:
            if each_item[0] == 'blue':
                blue_msg = [each_item[2], each_item[3], each_item[5]]
            if each_item[0] == 'nurse':
                nurse_msg = [each_item[2], each_item[3], each_item[5]]

        red_msg1 = Float64MultiArray(data=red_msg1)
        red_msg2 = Float64MultiArray(data=red_msg2)
        blue_msg = Float64MultiArray(data=blue_msg)
        nurse_msg = Float64MultiArray(data=nurse_msg)

        global pub_red1
        global pub_red2
        global pub_blue
        global pub_nurse
        pub_red1.publish(red_msg1)
        pub_red2.publish(red_msg2)
        pub_blue.publish(blue_msg)
        pub_nurse.publish(nurse_msg)
        # print(red_msg,blue_msg,nurse_msg)

    # if save_txt or save_img:
    #     s = f"\n{len(list(save_dir.glob('labels/*.txt')))} labels saved to {save_dir / 'labels'}" if save_txt else ''
    #     print(f"Results saved to {save_dir}{s}")

    print(f'Done. ({time.time() - t0:.3f}s)')


if __name__ == '__main__':

    # print(sys.path)
    # sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

    parser = argparse.ArgumentParser()
    # parser.add_argument('--weights', nargs='+', type=str, default='yolov5l.pt', help='model.pt path(s)')
    parser.add_argument('--weights', nargs='+', type=str, default='./trained_model/small/第一个/best.pt', help='model.pt path(s)')
    # parser.add_argument('--source', type=str, default='../my_dataset/images/train/', help='source')  # file/folder, 0 for webcam

    parser.add_argument('--source', type=str, default='1', help='source')  # file/folder, 0 for webcam
    # parser.add_argument('--source', type=str, default='data/video/1.mp4', help='source')  # file/folder, 0 for webcam

    parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
    parser.add_argument('--conf-thres', type=float, default=0.25, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='IOU threshold for NMS')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    # parser.add_argument('--device', default='cpu', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true', help='display results')
    parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
    parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')
    parser.add_argument('--nosave', action='store_true', help='do not save images/videos')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument('--update', action='store_true', help='update all models')
    parser.add_argument('--project', default='runs/detect', help='save results to project/name')
    parser.add_argument('--name', default='exp', help='save results to project/name')
    parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
    opt = parser.parse_args()
    print(opt)
    check_requirements(exclude=('pycocotools', 'thop'))


    rospy.init_node('topic_publisher')
    # BEGIN PUB
    global pub_red1
    global pub_red2
    global pub_blue
    global pub_nurse
    # pub_area = rospy.Publisher('robodetect_area', Int32)
    # pub_nurse_x = rospy.Publisher('robodetect_nurse_x', Int32)
    # pub_nurse_y = rospy.Publisher('robodetect_nurse_y', Int32)
    pub_red1 = rospy.Publisher('robodetect_pub_red1', Float64MultiArray)
    pub_red2 = rospy.Publisher('robodetect_pub_red2', Float64MultiArray)
    pub_blue = rospy.Publisher('robodetect_pub_blue', Float64MultiArray)
    pub_nurse = rospy.Publisher('robodetect_pub_nurse', Float64MultiArray)

    with torch.no_grad():
        if opt.update:  # update all models (to fix SourceChangeWarning)
            for opt.weights in ['yolov5s.pt', 'yolov5m.pt', 'yolov5l.pt', 'yolov5x.pt']:
                detect()
                strip_optimizer(opt.weights)
        else:
            detect()

    print()
