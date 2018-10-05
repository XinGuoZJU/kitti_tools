# for label
import os
from parseTrackletXML import parseXML as pa


path = "../kitti/data"
date = "2011_09_26"
label_save_path = "../kitti/label"

for i in range(92):    # 0-91
    dirs = os.path.join(os.path.join(path, date), '2011_09_26_drive_' + str(i).zfill(4) + '_sync')
    if os.path.exists(dirs):
        # check if the dir exists:
        save_label_dir = os.path.join(os.path.join(label_save_path, date), '2011_09_26_drive_' + str(i).zfill(4) + '_sync')
        if os.path.exists(save_label_dir):
            raise RuntimeError('Dir exists!')
        else:
            os.makedirs(save_label_dir)

        label_file = os.path.join(dirs, 'tracklet_labels.xml')
        labels = pa(label_file)
        for label in labels:
            for j in range(label.nFrames):
                each_frame = label.firstFrame+j
                save_file = os.path.join(save_label_dir, str(each_frame).zfill(10)+'.txt')
                with open(save_file, 'a') as save_op:
                    assert(label.rots[j][0] == label.rots[j][1] == 0)
                    # x,y,z,h,w,l,theta
                    save_str = label.objectType + ' ' + str(label.trans[j][0]) + ' ' + str(label.trans[j][1]) + ' ' + str(label.trans[j][2]) + \
                             ' ' + str(label.size[0]) + ' ' + str(label.size[1]) + ' ' + str(label.size[2]) + ' ' +str(label.rots[j][2]) + '\n'
                    save_op.write(save_str)
        '''
        # check the none frame
        bin_dir = os.path.join(os.path.join(path, date), '2011_09_26_drive_' + str(i).zfill(4) + '_sync/velodyne_points/data')
        bins = os.listdir(bin_dir)
        for item in bins:
            check_label = os.path.join(save_label_dir, item.split('.')[0] + '.txt')
            if not os.path.exists(check_label):
                f = open(check_label,'w')
                print(check_label)
                f.close()
        '''






'''

    objectType = None
    size = None  # len-3 float array: (height, width, length)
    firstFrame = None
    trans = None   # n x 3 float array (x,y,z)
    rots = None    # n x 3 float array (x,y,z)
    states = None  # len-n uint8 array of states
    occs = None    # n x 2 uint8 array  (occlusion, occlusion_kf)
    truncs = None  # len-n uint8 array of truncation
    amtOccs = None    # None or (n x 2) float array  (amt_occlusion, amt_occlusion_kf)
    amtBorders = None    # None (n x 3) float array  (amt_border_l / _r / _kf)
    nFrames = None

'''

'''
   1    type         Describes the type of object: 'Car', 'Van', 'Truck',
                     'Pedestrian', 'Person_sitting', 'Cyclist', 'Tram',
                     'Misc' or 'DontCare'
   1    truncated    Float from 0 (non-truncated) to 1 (truncated), where
                     truncated refers to the object leaving image boundaries
   1    occluded     Integer (0,1,2,3) indicating occlusion state:
                     0 = fully visible, 1 = partly occluded
                     2 = largely occluded, 3 = unknown
   1    alpha        Observation angle of object, ranging [-pi..pi]
   4    bbox         2D bounding box of object in the image (0-based index):
                     contains left, top, right, bottom pixel coordinates
   3    dimensions   3D object dimensions: height, width, length (in meters)
   3    location     3D object location x,y,z in camera coordinates (in meters)
   1    rotation_y   Rotation ry around Y-axis in camera coordinates [-pi..pi]
   1    score        Only for results: Float, indicating confidence in
                     detection, needed for p/r curves, higher is better.

'''
