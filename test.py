# analysis point cloud
import cv2
import numpy as np
import os

INDEX_LIST = [1, 2, 5, 9, 11, 13, 14, 15, 17, 18, 19, 20, 22, 23, 27, 28, 29, 32, 35, 36, 39, 46, 48, 51, 52, 56, 57, 59, 60, 61, 64, 70, 79, 84, 86, 87, 91]

class Projection():
    def __init__(self, path, select_list):
        self.path = path
        self.select_list = select_list
        self.calib_dict = dict()


    def run(self):
        self.__load_calib()
        for idx in INDEX_LIST:
            img = self.__load_img(idx)
            tracklet = self.__load_tracklet(idx)
            point_cloud = self.__load_bin(idx)
            obj_point = self.__filter(tracklet, point_cloud)
            obj_img = self.__D3toD2(img, obj_point)


    def __D3toD2(self, img, obj_point):
        obj_img = 0
        return obj_img


    def __filter(self, tracklet, point_cloud):
        print(len(tracklet))
        print(tracklet)

        obj_point = 0
        return obj_point


    def __load_calib(self):
        calib_path = os.path.join(self.path, 'kitti/2011_09_26/calib_velo_to_cam.txt')
        if not os.path.exists(calib_path):
            raise RuntimeError("Calib not exist!") 

        with open(calib_path, 'r') as read_op:
            lines = read_op.readlines()

            for line in lines:
                content = line.strip().split()
                if content[0] == 'R:':
                    R_list = list()
                    for i in range(1,10):
                        R_list.append(float(content[i]))
                    self.calib_dict['R'] = R_list
                elif content[0] == 'T:':
                    T_list = list()
                    for i in range(1,4):
                        T_list.append(float(content[i]))
                    self.calib_dict['T'] = T_list
                elif content[0] =='delta_f:':
                    f_list = list()
                    for i in range(1,3):
                        f_list.append(float(content[i]))
                    self.calib_dict['delta_f'] = f_list
                elif content[0] == 'delta_c:':
                    c_list = list()
                    for i in range(1,3):
                        c_list.append(float(content[i]))
                    self.calib_dict['delta_c'] = c_list


    def __load_img(self, idx):
        img_path = os.path.join(path, 'kitti/2011_09_26/2011_09_26_drive_0001_sync/image_03/data/'+ str(idx).zfill(10)+'.png')
        if not os.path.exists(img_path):
            raise RuntimeError("Image not exist!")

        img = cv2.imread(img_path)
        return img


    def __load_tracklet(self, idx):
        tracklet_path = os.path.join(self.path, 'label/2011_09_26/2011_09_26_drive_0001_sync/' + str(idx).zfill(10)+'.txt')
        if not os.path.exists(tracklet_path):
            raise RuntimeError("Tracklet not exist!") 
        
        with open(tracklet_path) as read_op:
            lines = read_op.readlines()
            tracklet_list = list()
            for line in lines:
                content = line.strip().split()
                if content[0] in self.select_list:
                    tracklet = list()
                    for i in range(1,7):
                        tracklet.append(float(content[i]))
                    tracklet_list.append(tracklet)
        return tracklet_list


    def __load_bin(self, idx):
        bin_path = os.path.join(self.path, 'kitti/2011_09_26/2011_09_26_drive_0001_sync/velodyne_points/data/' + str(idx).zfill(10)+'.bin')
        if not os.path.exists(bin_path):
            raise RuntimeError("Bin not exist!") 

        bin_data = np.fromfile(bin_path, dtype=np.float32)
        bin_re = bin_data.reshape((-1,4))
        
        return bin_re




path = "/home/kimjongkook/Desktop/kitti_tools"
a = Projection(path, ['Car'])
b = a.run()

