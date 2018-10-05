# analysis point cloud
import cv2
import numpy as np
import os

# Attention: In 2011_09_26_drive_0009_sync, the point cloud data (fram 177-180) do not exist! So I remove the label of them.
# COlOR_LIST refers  https://zhidao.baidu.com/question/368611693617526924.html

INDEX_LIST = [1, 2, 5, 9, 11, 13, 14, 15, 17, 18, 19, 20, 22, 23, 27, 28, 29, 32, 35, 36, 39, 46, 48, 51, 52, 56, 57, 59, 60, 61, 64, 70, 79, 84, 86, 87, 91]
COlOR_LIST = [[0,0,0], [256,256,256], [255,0,0], [255,102,0], [255,255,0], [0,255,0], [0,0,255], [255,0,255], [255,0,102], [102,0,255], [0,255,255], [153,255,0], [255,153,0], [255,51,0]] 


class Projection():
    def __init__(self, path, cam, select_list):
        self.__path = path
        self.__save_path = os.path.join(path, 'save')
        self.__cam = cam
        self.__select_list = select_list
        self.__calib_dict = dict()
        self.__load_calib()


    def run(self):
        self.__load_calib()
        for index in INDEX_LIST:
            self.subpath = '2011_09_26/2011_09_26_drive_' + str(index).zfill(4) + '_sync'
            save_path = os.path.join(os.path.join(self.__save_path, self.subpath),  'image_' + str(self.__cam).zfill(2))
            if not os.path.exists(save_path):
                os.makedirs(save_path)
            print(self.subpath)
            tracklet_list = os.listdir(os.path.join(self.__path, 'label/'+ self.subpath))

            for item in tracklet_list:
                idx = int(item.split('.')[0])
                img = self.__load_img(idx)
                tracklet = self.__load_tracklet(idx)
                if tracklet == []:
                    continue
                point_cloud = self.__load_bin(idx)
                point3D = self.__filter(tracklet, point_cloud)
                point2D = self.__D3toD2(point3D)
                new_img = self.__render(img, point2D)
                save_name = os.path.join(save_path, str(idx).zfill(10) + '.png')
                cv2.imwrite(save_name, img)


    def __render(self, img, points):
        for i in range(len(points)):
            point = points[i]
            for j in range(point.shape[1]):
                y = point[0,j]
                x = point[1,j]
                if x > -1 and x < 375 and y > -1 and y < 1242:
                    # print(i)
                    img[x,y] = np.array(COlOR_LIST[i])

        return img


    def __D3toD2(self, obj_point):
        R = self.__calib_dict['R']
        T = self.__calib_dict['T']

        P0 = np.column_stack((np.array(R).reshape((3,3)), np.asmatrix(T).T))
        Tr_vel_to_cam = np.row_stack([P0, np.array([0,0,0,1])])
        R_cam_to_rect = np.eye(4)
        R_cam_to_rect[:3,:3] = np.array(self.__calib_dict['R_rect']).reshape((3,3))
        P_cam_to_rect = np.array(self.__calib_dict['P_rect']).reshape((3,4))

        P = np.asmatrix(P_cam_to_rect) * np.asmatrix(R_cam_to_rect) * np.asmatrix(Tr_vel_to_cam)

        point2D_list = list()
        for point in obj_point:
            if len(point) == 0:
                continue
            p_matrix = np.array(point).T
            # velodyne and camera is different with y & z
            data3D = np.array([p_matrix[0], p_matrix[1], p_matrix[2], np.ones( p_matrix.shape[1])])
            data2D = P * np.asmatrix(data3D)

            point2D = np.row_stack((data2D[0]/data2D[2], data2D[1]/data2D[2]))
            int_point2D = np.rint(point2D).astype(np.int)
            point2D_list.append(int_point2D)

        return point2D_list


    def __is_point_in_track(self, point, track):
        #print(track)
        x = track[0]
        y = track[1]
        z = track[2]
        h = track[3]
        w = track[4]
        l = track[5]
        theta = track[6]
        rote_point = self.__rote(point, [x, y, z], -theta)

        if rote_point[0] < x + 0.5 * l and rote_point[0] > x - 0.5 * l and \
              rote_point[1] < y + 0.5 * w and rote_point[1] > y - 0.5 * w and \
              rote_point[2] < z + 0.5 * h and rote_point[2] > z - 0.5 * h:
            return True
        else:
            return False


    def __rote(self, point, center, theta0):
        theta = theta0 * np.pi/180
        point_ce = [point[i]-center[i] for i in range(2)]
        rote_point = [point_ce[0]*np.cos(theta) - point_ce[1]*np.sin(theta) + center[0], \
                    point_ce[1]*np.cos(theta) + point_ce[0]*np.sin(theta) + center[1], \
                    point[2]]

        return rote_point


    def __filter(self, tracklet, point_cloud):
        obj_point = [[] for i in range(len(tracklet))]
        for point in point_cloud:
            for i in range(len(tracklet)):
                if self.__is_point_in_track(point, tracklet[i]):
                    obj_point[i].append(point)

        return obj_point


    def __load_calib(self):
        calib_velo_to_cam_path = os.path.join(self.__path, 'data/2011_09_26/calib_velo_to_cam.txt')
        if not os.path.exists(calib_velo_to_cam_path):
            raise RuntimeError("Calib_velo_to_cam not exist!") 

        with open(calib_velo_to_cam_path, 'r') as read_op:
            lines = read_op.readlines()
            for line in lines:
                content = line.strip().split()
                if content[0] == 'R:':
                    R_list = list()
                    for i in range(1,10):
                        R_list.append(float(content[i]))
                    self.__calib_dict['R'] = R_list
                elif content[0] == 'T:':
                    T_list = list()
                    for i in range(1,4):
                        T_list.append(float(content[i]))
                    self.__calib_dict['T'] = T_list
                elif content[0] =='delta_f:':
                    f_list = list()
                    for i in range(1,3):
                        f_list.append(float(content[i]))
                    self.__calib_dict['delta_f'] = f_list
                elif content[0] == 'delta_c:':
                    c_list = list()
                    for i in range(1,3):
                        c_list.append(float(content[i]))
                    self.__calib_dict['delta_c'] = c_list


        calib_cam_to_cam_path = os.path.join(self.__path, 'data/2011_09_26/calib_cam_to_cam.txt')
        if not os.path.exists(calib_cam_to_cam_path):
            raise RuntimeError("Calib_cam_to_cam not exist!") 

        with open(calib_cam_to_cam_path, 'r') as read_op:
            lines = read_op.readlines()
            for line in lines:
                content = line.strip().split()
                if content[0] == 'R_rect_' + str(self.__cam).zfill(2) + ':':
                    R_list = list()
                    for i in range(1,10):
                        R_list.append(float(content[i]))
                    self.__calib_dict['R_rect'] = R_list
                elif content[0] == 'P_rect_' + str(self.__cam).zfill(2) + ':':
                    P_list = list()
                    for i in range(1,13):
                        P_list.append(float(content[i]))
                    self.__calib_dict['P_rect'] = P_list


    def __load_img(self, idx):
        img_path = os.path.join(path, 'data/'+ self.subpath + '/image_' + str(self.__cam).zfill(2) + '/data/'+ str(idx).zfill(10)+'.png')
        if not os.path.exists(img_path):
            raise RuntimeError("Image not exist!")
        img = cv2.imread(img_path)

        return img


    def __load_tracklet(self, idx):
        tracklet_path = os.path.join(self.__path, 'label/'+ self.subpath + '/' + str(idx).zfill(10)+'.txt')
        if not os.path.exists(tracklet_path):
            raise RuntimeError("Tracklet not exist!") 
        
        with open(tracklet_path) as read_op:
            lines = read_op.readlines()
            tracklet_list = list()
            for line in lines:
                content = line.strip().split()
                if content[0] in self.__select_list:
                    tracklet = list()
                    # x,y,z,h,w,l,theta
                    for i in range(1,8):
                        tracklet.append(float(content[i]))
                    tracklet_list.append(tracklet)

        return tracklet_list


    def __load_bin(self, idx):
        bin_path = os.path.join(self.__path, 'data/'+ self.subpath + '/velodyne_points/data/' + str(idx).zfill(10)+'.bin')
        if not os.path.exists(bin_path):
            print(idx)
            raise RuntimeError("Bin not exist!") 

        bin_data = np.fromfile(bin_path, dtype=np.float32)
        bin_re = bin_data.reshape((-1,4))
        
        return bin_re



if __name__ == '__main__':
    path = "/home/kimjongkook/Desktop/kitti"
    a = Projection(path, 2, ['Car'])
    b = a.run()

