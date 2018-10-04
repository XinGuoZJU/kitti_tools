# for label
from parseTrackletXML import parseXML as pa

filename = "/home/kimjongkook/Desktop/kitti/2011_09_26/2011_09_26_drive_0001_sync/tracklet_labels.xml"
guoxin = pa(filename)

# for data
import pykitti
import numpy as np

basedir = '/home/kimjongkook/Desktop/kitti'
date = '2011_09_26'
drive = '0001'

data = pykitti.raw(basedir, date, drive, frames=range(0, 50, 5))

point_velo = np.array([0,0,0,1])
point_cam0 = data.calib.T_cam0_velo.dot(point_velo)

point_imu = np.array([0,0,0,1])
point_w = [o.T_w_imu.dot(point_imu) for o in data.oxts]

for cam0_image in data.cam0:
    # do something
    pass

cam2_image, cam3_image = data.get_rgb(3)
