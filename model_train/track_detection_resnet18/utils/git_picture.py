import os
import cv2
import numpy as np
from time import time
from hobot_vio import libsrcampy


DATASET_NAME = "track_dataset/"  # 新建的文件夹名称
SPEED = 15 # 每秒5张
NUM = 500 # 一共采集500张照片
DATASET_PATH = "./"  # 保存的小文件夹


def mipi_img2cv2_img(mipi_img, height, width):
    # 转化为numpy数组
    mipi_img = np.frombuffer(mipi_img, dtype=np.uint8)

    # 按照nv12的格式进行排布
    mipi_img = mipi_img.reshape(int(1.5*height), width)

    # 利用cv2的函数进行格式转换
    cv2_img = cv2.cvtColor(mipi_img, cv2.COLOR_YUV420SP2BGR) 
    return cv2.cvtColor(cv2_img, cv2.COLOR_BGR2RGB)

def main():
    ## 创建文件夹
    folder = os.path.exists(DATASET_PATH + DATASET_NAME)

    # 如果不存在，则创建文件夹并开始图片采集
    if not folder:
        os.makedirs(DATASET_PATH + DATASET_NAME + "224_img/")
        os.makedirs(DATASET_PATH + DATASET_NAME + "224_png/")
        os.makedirs(DATASET_PATH + DATASET_NAME + "512_img/")
        os.makedirs(DATASET_PATH + DATASET_NAME + "512_png/")
        os.makedirs(DATASET_PATH + DATASET_NAME + "2K_png/")
        print("DATASET " + DATASET_PATH + DATASET_NAME + " created Successful!")

        ## 创建VPS和camera等对象，开始图像采集
        # camera对象
        cap_mipi = libsrcampy.Camera()
        height1, width1 = 512, 512
        height2, width2 = 720, 1280
        height, width = 1440, 2560
        ret = cap_mipi.open_cam(0, -1, 30, [width1, width2, width], [height1, height2, height])
        print("cap_mipi.open_cam State: ", end="")
        print(ret)

        # vps对象
        vps = libsrcampy.Camera()
        ret = vps.open_vps(1, 1, 512, 512, 224, 224)
        print("vps State: ", end="")
        print(ret)

        # 图像采集
        for i in range(NUM):
            for q in range(int(30/SPEED) - 1):
                cap_mipi.get_img(2, width1, height1)
            time_begin = time()

            # 获取720p的图, 四合一像素
            # mipi_data_2K = cap_mipi.get_img(2, width2, height2)
            # png_2K = mipi_img2cv2_img(mipi_data_2K, height2, width2)
            # cv2.imwrite(DATASET_PATH + DATASET_NAME + "2K_png/%04d.png"%i, png_2K)
            # 获取512的图
            mipi_data_512 = cap_mipi.get_img(2, width1, height1)

            # 保存512_img
            fo = open(DATASET_PATH + DATASET_NAME + "512_img/%04d.img"%i, "wb")
            fo.write(mipi_data_512)
            fo.close()

            # 保存512_png
            png_512 = mipi_img2cv2_img(mipi_data_512, height1, width1)
            cv2.imwrite(DATASET_PATH + DATASET_NAME + "512_png/%04d.png"%i, png_512)

            # 保存224_img
            vps.set_img(mipi_data_512)
            mipi_data_224 = vps.get_img(2, 224, 224)
            fo = open(DATASET_PATH + DATASET_NAME + "224_img/%04d.img"%i, "wb")
            fo.write(mipi_data_224)
            fo.close()

            # 保存224_png
            png_224 = mipi_img2cv2_img(mipi_data_224, 224, 224)
            cv2.imwrite(DATASET_PATH + DATASET_NAME + "224_png/%04d.png"%i, png_224)

            # 打印采集信息
            print(DATASET_PATH + DATASET_NAME + "224_png/%04d.png Saved"%i, end="  ")
            print("Cost = %.2f ms"%(1000*(time()-time_begin)))
        # 回收
        cap_mipi.close_cam()
    # 如果存在这个文件夹，则报错并退出
    else:
        print("DATASET " + DATASET_NAME + " Exists, please rename your new Dataset")

if __name__ == "__main__":
    main()
