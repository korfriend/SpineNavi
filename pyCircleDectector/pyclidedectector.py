import numpy as np
import cv2 as cv
import math
from PIL import Image
import glob
#-------------------- detector hyperparameter -----------------------------
params = cv.SimpleBlobDetector_Params()
params.filterByArea = True
params.filterByCircularity = True
params.filterByConvexity = False
params.filterByInertia = True
params.filterByColor = False
params.minArea = 3000 # The size of the blob filter to be applied. If the corresponding value is increased, small circles are not detected. 
params.maxArea = 80000
params.minCircularity = 0.01 # 1 >> it detects perfect circle. Minimum size of center angle
params.minInertiaRatio = 0.2 # 1 >> it detects perfect circle. short/long axis
params.minRepeatability = 3
params.minDistBetweenBlobs = 0.01
#-------------------------------------------------------------------------

#-------------------- text hyperparameter ------------------------------
fontFace = cv.FONT_HERSHEY_SIMPLEX
fontScale = 0.5
color = (255, 255, 255)
thickness = 2
lineType = cv.LINE_AA
#-------------------------------------------------------------------------
#------------------Mouse Click Event--------------------------------------
current_2D_pos = []
sorted_position = []

def mouse_click(event, x, y, flags, param):
    if event == cv.EVENT_LBUTTONDOWN:
        # print(current_2D_pos)
        distance_between_circle2mouse = []
        for circle_x, circle_y in current_2D_pos:
            distance_between_circle2mouse.append((circle_x - x)**2 + (circle_y - y)**2)
        min_value = min(distance_between_circle2mouse)
        min_index = distance_between_circle2mouse.index(min_value)
        print(current_2D_pos[min_index])
        sorted_position.append(current_2D_pos[min_index])
#---------------------------------------------------------------------------------------------

#------------------ create detector ------------------------------------------------
detector = cv.SimpleBlobDetector_create(params)
    
PATTERN_SIZE = (6,3) # 18 circle exist
UNIT_SIZE = 47.8125 # distance between circles // unit is millimeter

IMG_PATH_PRE = "../data/c-arm 2023-05-20/*.png"
image_names_pre = glob.glob(IMG_PATH_PRE)
print(image_names_pre)

IMG_PATH_ORI = "../data/c-arm 2023-05-20/*.png"
image_names_ori = glob.glob(IMG_PATH_ORI)
print(image_names_ori)

for j in range(len(image_names_pre)):
    
    imgPre = cv.imread(image_names_pre[j],cv.IMREAD_GRAYSCALE)
    H, W = imgPre.shape[:2] 

    img_name = image_names_pre[j].split("\\")
    # detected circle
    keyPoints = detector.detect(imgPre)
    print(img_name[1])
    # visualize circle in original image
    
    imgOri = cv.imread(image_names_ori[j],cv.IMREAD_GRAYSCALE)
    imgOri = cv.cvtColor(imgOri, cv.COLOR_BGR2RGB)
    
    for i in range(len(keyPoints)):
        keypoint = keyPoints[i]
        
        x = round(keypoint.pt[0])
        y = round(keypoint.pt[1])
        s = keypoint.size
        r = int(math.floor(s / 2))
        
        # draw circle
        cv.circle(imgOri, (x, y), r, (0, 0, 255), 1) 
        cv.putText(imgOri, str(x) + "," + str(y),(x,y), fontFace, fontScale, color, thickness, lineType)
        current_2D_pos.append([keypoint.pt[0],keypoint.pt[1]])
        
        
    # visualize 
    cv.imshow(img_name[1],imgOri) 
    cv.setMouseCallback(img_name[1], mouse_click)
    cv.waitKey(0)
    
            
    # save file by cv2.FileStorage()        
    # fs = cv.FileStorage("./EHmin/xml/" + str(j + 7817)+'.txt', cv.FILE_STORAGE_WRITE)
    fs_name = img_name[1].split(".")
    fs = cv.FileStorage("../data/c-arm 2023-05-20/" + fs_name[0] +'.txt', cv.FILE_STORAGE_WRITE)
    fs.write("undist_circle_center_pos", np.array(sorted_position))
    fs.release()

        
    current_2D_pos.clear()
    sorted_position.clear()
    
    
cv.waitKey(0)
