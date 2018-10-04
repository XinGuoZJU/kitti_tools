# for label
import os
from parseTrackletXML import parseXML as pa

#guoxin = pa(filename)

#path = ''
path = "/home/kimjongkook/Desktop/kitti"
date = os.path.join(path, '2011_09_26')

for i in range(92):    # 0-91
    dirs = os.path.join(date, '2011_09_26_drive_' + str(i).zfill(4) + '_sync')
    if os.path.exists(dirs):
        label_file = os.path.join(dirs, 'tracklet_labels.xml')
        labels = pa(label_file)
        for label in labels:
            print(label.nFrames)






