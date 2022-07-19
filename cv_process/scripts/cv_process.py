#!/usr/bin/env python  
from tkinter import TRUE
from tkinter import FALSE
import numpy as np
import cv2
import math
import os
import rospy
import sys
sys.path.append('/home/cyh/jaka_robot/jaka_robot/devel/lib/python3/dist-packages')
from teris_msgs.srv import *
os.environ["CUDA_VISIBLE_DEVICES"] = "0"


def isPointInRect(ptf4, ptf, fAngle):

    nQuadrant = [0]*4
    fAngle *= np.pi/180.0*(-1)

    for idx in range(4):
        fDifx = ptf[0] - ptf4[idx][0]
        fDify = ptf[1] - ptf4[idx][1]
        nDifx = fDifx*math.cos(fAngle)-fDify*math.sin(fAngle)
        nDify = fDifx*math.sin(fAngle)+fDify*math.cos(fAngle)

        # 第一象限
        if(nDifx >= 0 and nDify >= 0):
            nQuadrant[0] += 1
        # 第二象限
        if(nDifx < 0 and nDify >= 0):
            nQuadrant[1] += 1
        # 第三象限
        if(nDifx < 0 and nDify < 0):
            nQuadrant[2] += 1
        # 第四象限
        if(nDifx > 0 and nDify < 0):
            nQuadrant[3] += 1
    # 判断四个向量是否在相邻的两个象限
    firstIdx = -1
    secIdx = -1
    countNum = 0
    for idx in range(4):
        if(nQuadrant[idx] != 0):
            if(firstIdx == -1):
                firstIdx = idx
            elif (secIdx == -1 and firstIdx != -1):
                secIdx = idx

            countNum += 1

    if(countNum <= 2):
        if(abs(firstIdx - secIdx) == 1 or abs(firstIdx - secIdx) == 3 or (countNum == 1 and (firstIdx == -1 or secIdx == -1))):
            return FALSE
    return TRUE


def getDist_P2P(Point0, PointA):
    distance = math.pow((Point0[0]-PointA[0]), 2) + \
        math.pow((Point0[1]-PointA[1]), 2)
    distance = math.sqrt(distance)
    return distance


def NMS(dets, distance):  # dets:输入矩形框;distance:置信距离
    center = (dets[:, 0])
    WH = (dets[:, 1])
    width = [x[0] for x in WH]
    height = [x[1] for x in WH]
    areas = [x*y for x, y in zip(width, height)]
    # print(areas)
    angle = (dets[:, 2])

    keep = []
    for i in range(len(angle)):
        keep.append(i)
    # print(keep)
    for i in range(len(angle)):
        for j in range(i+1, len(angle)):
            temp = getDist_P2P(center[i], center[j])
            if temp <= distance:
                if areas[i] >= areas[j]:
                    keep[i] = j
                else:
                    keep[j] = i
    keep = np.unique(keep)
    # print(keep)
    return keep


def minRect(imgray, imgcopy):  # 输出框选的信息（包括坐标，倾斜角度）
    result = []

    contours, hierarchy = cv2.findContours(
        imgray, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    imgcopy = cv2.drawContours(imgcopy, contours, -1, (0, 0, 255), 2)

    # xianShi(imgcopy)

    # for cnt in contours:
    #     epsilon = 0.01*cv2.arcLength(cnt, False)
    #     approx1 = cv2.approxPolyDP(cnt, epsilon, False)
    #     imgcopy = cv2.polylines(imgcopy, [approx1], True, (255, 255, 0), 2)

    # xianShi(imgcopy)

    for cnt in contours:

        rect = cv2.minAreaRect(cnt)
        size = rect[1]
        area = size[0]*size[1]

        if area >= 1000 and area <= 7000:
            result.append(rect)
            #print(area)
            P = cv2.boxPoints(rect)
            P = np.int0(P)
            imgcopy = cv2.drawContours(imgcopy, [P], -1, (0, 0, 255), 2)

    result = np.array(result)
    # print(result)
    result_INDEX = NMS(result, 5)
    result_0 = result[result_INDEX]
    # print(result_0)

    for rect in result_0:
        P = cv2.boxPoints(rect)
        # print(rect)
        P = np.int0(P)
        imgcopy = cv2.drawContours(imgcopy, [P], -1, (0, 255, 255), 2)
    # xianShi(imgcopy)

    return result_0


def shaiXuan(imgcopy, min, max):  # 转换为HSV
    img_hsv = cv2.cvtColor(imgcopy, cv2.COLOR_BGR2HSV)
    hsv_mask = cv2.inRange(img_hsv, np.array(min), np.array(max))

    return hsv_mask


def xianShi(img):
    cv2.imshow('img', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def bianYan(img_gray):
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

    img_blur = cv2.GaussianBlur(img_gray, (5, 5), 0)
    edges = cv2.Canny(image=img_blur, threshold1=70, threshold2=160)
    #xianShi(edges)
    edges = cv2.dilate(edges, kernel)
    # edges = cv2.dilate(edges, kernel)
    edges = cv2.erode(edges, kernel, iterations=1)

    edges = cv2.ximgproc.thinning(edges, 1)
    #xianShi(edges)

    return edges


def rotated_change(rotated_rect):
    center = rotated_rect[:, 0]
    size = rotated_rect[:, 1]
    angle = rotated_rect[:, 2]
    angle *= np.pi/180.0*(-1)
    width = [x[0] for x in size]
    height = [x[1] for x in size]
    rtf4 = [[[], [], [], []]]

    for i in range(len(center)-1):
        rtf4.append([[], [], [], []])  # 初始化rtf4列表

    for i in range(len(center)):
        for j in range(2):
            for k in range(2):
                rtf4[i][j*2+k] = (center[i][0]+pow(-1, j)*0.5*width[i]*math.cos(angle[i])+pow(-1, k)*0.5*height[i]*math.sin(angle[i]),
                                  center[i][1]+pow(-1, j)*0.5*width[i]*math.sin(angle[i])-pow(-1, k)*0.5*height[i]*math.cos(angle[i]))

    return rtf4


def PD_shape(rotated_rect, imgcopy_color, CL):
    '''
    ptf4t:输入矩形(四个点) 
    img_copy:原图 
    imgcopy_color:筛选出来的图片
    color:颜色
    return: 列表索引号
    '''
    angle = rotated_rect[:, 2]
    ptf4 = rotated_change(rotated_rect)
    ptf4 = np.int0(ptf4)
    imgcopy_color_row = np.size(imgcopy_color, 0)  # 计算 X 的行数
    imgcopy_color_col = np.size(imgcopy_color, 1)

    sum = 0
    keep = []
    for i in range(len(rotated_rect)):
        if(sum >= CL):
            sum = 0
        for j in range(ptf4[i][2][0],ptf4[i][1][0]):  # 行坐标
            if(sum >= CL):
                continue
            for k in range(ptf4[i][0][1],ptf4[i][3][1]):  # 列坐标
                if(sum >= CL):
                    continue
                if (imgcopy_color[k][j] == 255):
                    if(isPointInRect(ptf4[i], (j, k), angle[i])):
                        sum += 1
                        if(sum >= CL):
                            keep.append(i)
    for i in range(len(rotated_rect)):
        angle[i] = angle[i]/np.pi*(-1)*180
    return keep


# 获取抓取点的最终坐标，用以判断，预计和PD_shape一起使用
def retrieve_coordinate(keep, Rotated_rect, case_color, color_mask):
    '''
    keep:接受的矩形索引
    Rotated_rect:存储待处理的旋转rect矩阵
    case_color:所使用的目标颜色
    case 0:黄色L
    case 1:紫色L
    case 2:蓝色Z
    case 3:绿色Z
    case 4:棕色 山
    case 5:橙色 田
    case 6:红色 一
    color_mask:颜色筛选二值化图像
    return:  返回等待夹取坐标点
    '''
    target_point = []
    target_angle = []
    center = Rotated_rect[keep, 0]
    size = Rotated_rect[keep, 1]
    angle = [90-x for x in Rotated_rect[keep, 2]]
    angle = [x*np.pi/180.0 for x in angle]
    P = []
    for cnt in Rotated_rect[keep]:
        P.append(np.int0(cv2.boxPoints(cnt)))
    temp = []
    PD_angle = False
    for i in range(len(center)):
        temp.clear()
        if case_color <= 4:
            if size[i][0] < size[i][1]:
                PD_angle = True
                temp_size = (size[i][0]/2, size[i][1]/3)
                for j in range(2):
                    for k in range(-1, 2):
                        temp_center = (center[i][0]-(-1)*k*(1/3)*size[i][1]*math.cos(angle[i])+pow(-1, j)*(1/4)*size[i][0]*math.sin(
                            angle[i]), center[i][1]+(-1)*k*(1/3)*size[i][1]*math.sin(angle[i])+pow(-1, j)*(1/4)*size[i][0]*math.cos(angle[i]))
                        temp_angle = 90 - angle[i]*180/np.pi
                        temp.append([temp_center, temp_size, temp_angle])
            else:
                PD_angle = False
                temp_size = (size[i][0]/3, size[i][1]/2)
                for j in range(2):
                    for k in range(-1, 2):
                        temp_center = (center[i][0]+pow(-1, j)*0.25*size[i][1]*math.cos(angle[i])+(-1)*k*(1/3)*size[i][0]*math.sin(
                            angle[i]), center[i][1]-pow(-1, j)*0.25*size[i][1]*math.sin(angle[i])+(-1)*k*(1/3)*size[i][0]*math.cos(angle[i]))
                        temp_angle = 90 - angle[i]*180/np.pi
                        temp.append([temp_center, temp_size, temp_angle])
        if case_color <= 4:
            temp_0 = np.array(temp)
            # PD_keep 存储子三角形中是否有颜色
            PD_keep_0 = PD_shape(temp_0, color_mask, 10)
            PD_keep = []
            for i in range(6):
                PD_keep.append(False)
            PD_keep = np.array(PD_keep)
            PD_keep[PD_keep_0] = True

        if case_color <= 1:  # L型
            if PD_keep[0] ^ PD_keep[2]:
                if PD_keep[0]:
                    target_point.append(temp[0][0])
                    if PD_angle:
                        target_angle.append(-1*temp_angle)
                    else:
                        target_angle.append(90+(-1)*temp_angle)
                else:
                    target_point.append(temp[2][0])
                    if PD_angle:
                        target_angle.append(180-1*temp_angle)
                    else:
                        target_angle.append(-90+(-1)*temp_angle)
            else:
                if PD_keep[3]:
                    target_point.append(temp[3][0])
                    if PD_angle:
                        target_angle.append(-1*temp_angle)
                    else:
                        target_angle.append(90+(-1)*temp_angle)
                else:
                    target_point.append(temp[5][0])
                    if PD_angle:
                        target_angle.append(180-1*temp_angle)
                    else:
                        target_angle.append(-90+(-1)*temp_angle)
        elif case_color <= 3:  # Z型
            if PD_angle:
                target_angle.append(-1*temp_angle)
            else:
                target_angle.append(90+(-1)*temp_angle)
            if PD_keep[0]:
                target_point.append(temp[0][0])
            else:
                target_point.append(temp[3][0])
        elif case_color == 4:
            if PD_keep[0] ^ PD_keep[1]:
                target_point.append(temp[5][0])
                if PD_angle:
                    target_angle.append(180 - 1*temp_angle)
                else:
                    target_angle.append(180 + 90+(-1)*temp_angle)
            elif PD_keep[2] ^ PD_keep[1]:
                target_point.append(temp[5][0])
                if PD_angle:
                    target_angle.append(180 - 1*temp_angle)
                else:
                    target_angle.append(180 + 90+(-1)*temp_angle)
            else:
                target_point.append(temp[0][0])
                if PD_angle:
                    target_angle.append(-1*temp_angle)
                else:
                    target_angle.append(90+(-1)*temp_angle)
        if case_color == 5:
            target_point.append((P[i][3]+np.int0(center[i]))/2)
            target_angle.append(-90 + angle[i]*180/np.pi)
        if case_color == 6:
            if size[i][0] > size[i][1]:
                target_point.append((center[i][0]+0.375*size[i][0]*math.sin(
                    angle[i]), center[i][1]+0.375*size[i][0]*math.cos(angle[i])))
                target_angle.append(angle[i]*180/np.pi)
            else:
                target_point.append((center[i][0]-0.375*size[i][1]*math.cos(
                    angle[i]), center[i][1]+0.375*size[i][1]*math.sin(angle[i])))
                target_angle.append(-90+angle[i]*180/np.pi)

    return target_point, target_angle


if __name__ == '__main__':

    result_final = []

    img_0 = cv2.imread('/home/cyh/jaka_robot/jaka_robot/src/cv_process/scripts/teris.png')
    bbox = cv2.selectROI(img_0, False)
    cut = img_0[bbox[1]:bbox[1]+bbox[3], bbox[0]:bbox[0]+bbox[2]]
    bbox = np.array(bbox)
    cv2.imshow('img', cut)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    cv2.imwrite('cut.png', cut)
    img = cv2.imread('cut.png')
    # img = cv2.resize(img, dsize=None, fx=0.5, fy=0.5)
    imgcopy = img.copy()
    imgcopy_0 = img.copy()
    xianShi(imgcopy)

    img_gray = cv2.cvtColor(imgcopy, cv2.COLOR_BGR2GRAY)
    img_edge = bianYan(imgcopy)
    result = minRect(img_edge, imgcopy)
# ------------------------------------------------------------------------------------------------颜色提取
    imgcopy_red_1 = shaiXuan(imgcopy_0, [0, 120, 90], [4, 215, 140])  # 红色提取1*
    imgcopy_red_0 = shaiXuan(imgcopy_0, [170, 105, 120], [185, 205, 145])
    imgcopy_red = cv2.bitwise_or(imgcopy_red_1, imgcopy_red_0)
    imgcopy_red = cv2.medianBlur(imgcopy_red, 3)  # 中值滤波
    imgcopy_red_thin = cv2.ximgproc.thinning(imgcopy_red, 1)
    # xianShi(imgcopy_red_thin)

    keep_0 = PD_shape(result, imgcopy_red_thin, 45)
    target_point, target_angle = retrieve_coordinate(
        keep_0, result, 6, imgcopy_red_thin)
    result_temp_red = [[(x[0]+bbox[0],x[1]+bbox[1]), y]
                          for x, y in zip(target_point, target_angle)]
    imgcopy1=imgcopy.copy()
    for i in range(len(target_point)):
        points = target_point[i]
        points = np.int0(points)
        imgcopy1 = cv2.circle(imgcopy1, points, 5, (255, 255, 0), -1)
    xianShi(imgcopy1)
    

    imgcopy_green = shaiXuan(imgcopy_0, [40, 90, 71], [66, 213, 132])  # 绿色提取
    imgcopy_green = cv2.medianBlur(imgcopy_green, 3)  # 中值滤波
    # xianShi(imgcopy_green)
    imgcopy_green_thin = cv2.ximgproc.thinning(imgcopy_green, 1)
    # xianShi(imgcopy_green_thin)

    keep_1 = PD_shape(result, imgcopy_green_thin, 45)
    target_point, target_angle = retrieve_coordinate(
        keep_1, result, 3, imgcopy_green_thin)
    result_temp_green = [[(x[0]+bbox[0],x[1]+bbox[1]), y]
                          for x, y in zip(target_point, target_angle)]
                          
    imgcopy1=imgcopy.copy()
    for i in range(len(target_point)):
        points = target_point[i]
        points = np.int0(points)
        imgcopy1 = cv2.circle(imgcopy1, points, 5, (255, 255, 0), -1)
    xianShi(imgcopy1)

    imgcopy_blue = shaiXuan(imgcopy_0, [90, 200, 100], [110, 255, 200])  # 蓝色提取
    imgcopy_blue = cv2.medianBlur(imgcopy_blue, 3)  # 中值滤波
    #xianShi(imgcopy_blue)
    imgcopy_blue_thin = cv2.ximgproc.thinning(imgcopy_blue, 1)
    #xianShi(imgcopy_blue_thin)

    keep_2 = PD_shape(result, imgcopy_blue_thin, 45)
    target_point, target_angle = retrieve_coordinate(
        keep_2, result, 2, imgcopy_blue_thin)
    result_temp_blue = [[(x[0]+bbox[0],x[1]+bbox[1]), y]
                          for x, y in zip(target_point, target_angle)]
    imgcopy1=imgcopy.copy()
    for i in range(len(target_point)):
        points = target_point[i]
        points = np.int0(points)
        imgcopy1 = cv2.circle(imgcopy1, points, 5, (255, 255, 0), -1)
    xianShi(imgcopy1)

    imgcopy_yellow = shaiXuan(imgcopy_0, [22, 150, 46], [33, 255, 255])  # 黄色提取
    imgcopy_yellow = cv2.medianBlur(imgcopy_yellow, 3)  # 中值滤波
    #xianShi(imgcopy_yellow)
    imgcopy_yellow_thin = cv2.ximgproc.thinning(imgcopy_yellow, 1)
    #xianShi(imgcopy_yellow_thin)

    keep_3 = PD_shape(result, imgcopy_yellow_thin, 45)
    target_point, target_angle = retrieve_coordinate(
        keep_3, result, 0, imgcopy_yellow_thin)
    result_temp_yellow = [[(x[0]+bbox[0],x[1]+bbox[1]), y]
                          for x, y in zip(target_point, target_angle)]
                          
    imgcopy1=imgcopy.copy()
    for i in range(len(target_point)):
        points = target_point[i]
        points = np.int0(points)
        imgcopy1 = cv2.circle(imgcopy1, points, 5, (255, 255, 0), -1)
    xianShi(imgcopy1)

    imgcopy_brown_0 = shaiXuan(imgcopy_0, [0, 0, 10], [20, 115, 84])  # 棕色提取*
    imgcopy_brown_1 = shaiXuan(imgcopy_0, [150, 10, 20], [180, 85, 55])
    imgcopy_brown = cv2.bitwise_or(imgcopy_brown_0, imgcopy_brown_1)
    imgcopy_brown = cv2.medianBlur(imgcopy_brown, 3)  # 中值滤波
    #xianShi(imgcopy_brown)
    imgcopy_brown_thin = cv2.ximgproc.thinning(imgcopy_brown, 1)
    #xianShi(imgcopy_brown_thin)

    keep_4 = PD_shape(result, imgcopy_brown_thin, 45)
    
    keep_4=[x for x in keep_4 if x not in keep_0]
    keep_4=[x for x in keep_4 if x not in keep_1]
    keep_4=[x for x in keep_4 if x not in keep_2]
    keep_4=[x for x in keep_4 if x not in keep_3]
    
    target_point, target_angle = retrieve_coordinate(
        keep_4, result, 4, imgcopy_brown_thin)
    result_temp_brown = [[(x[0]+bbox[0],x[1]+bbox[1]), y]
                          for x, y in zip(target_point, target_angle)]
    imgcopy1=imgcopy.copy()
    for i in range(len(target_point)):
        points = target_point[i]
        points = np.int0(points)
        imgcopy1 = cv2.circle(imgcopy1, points, 5, (255, 255, 0), -1)
    xianShi(imgcopy1)

    imgcopy_purple = shaiXuan(imgcopy_0,  [100, 39, 30], [147, 140, 110])  # 紫色提取
    imgcopy_purple = cv2.medianBlur(imgcopy_purple, 3)  # 中值滤波
    #xianShi(imgcopy_purple)
    imgcopy_purple_thin = cv2.ximgproc.thinning(imgcopy_purple, 1)
    #xianShi(imgcopy_purple_thin)

    keep_5 = PD_shape(result, imgcopy_purple_thin, 45)
    
    keep_5=[x for x in keep_5 if x not in keep_0]
    keep_5=[x for x in keep_5 if x not in keep_1]
    keep_5=[x for x in keep_5 if x not in keep_2]
    keep_5=[x for x in keep_5 if x not in keep_3]
    keep_5=[x for x in keep_5 if x not in keep_4]
    
    target_point, target_angle = retrieve_coordinate(
        keep_5, result, 1, imgcopy_purple_thin)
    result_temp_purple = [[(x[0]+bbox[0],x[1]+bbox[1]), y]
                          for x, y in zip(target_point, target_angle)]
    imgcopy1=imgcopy.copy()
    for i in range(len(target_point)):
        points = target_point[i]
        points = np.int0(points)
        imgcopy1 = cv2.circle(imgcopy1, points, 5, (255, 255, 0), -1)
    xianShi(imgcopy1)

    imgcopy_orange = shaiXuan(imgcopy_0,  [7, 140, 119], [12, 245, 180])  # 橙色提取
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    imgcopy_orange = cv2.medianBlur(imgcopy_orange, 3)  # 中值滤波
    #xianShi(imgcopy_orange)
    imgcopy_orange_thin = cv2.erode(imgcopy_orange, kernel, iterations=1)
    #xianShi(imgcopy_orange_thin)
    keep_6 = PD_shape(result, imgcopy_orange_thin, 45)
    
    keep_6=[x for x in keep_6 if x not in keep_0]
    keep_6=[x for x in keep_6 if x not in keep_1]
    keep_6=[x for x in keep_6 if x not in keep_2]
    keep_6=[x for x in keep_6 if x not in keep_3]
    keep_6=[x for x in keep_6 if x not in keep_4]
    keep_6=[x for x in keep_6 if x not in keep_5]
    
    
    target_point, target_angle = retrieve_coordinate(
        keep_6, result, 5, imgcopy_orange_thin)
    result_temp_orange = [[(x[0]+bbox[0],x[1]+bbox[1]), y]
                          for x, y in zip(target_point, target_angle)]
    imgcopy1=imgcopy.copy()
    for i in range(len(target_point)):
        points = target_point[i]
        points = np.int0(points)
        imgcopy1 = cv2.circle(imgcopy1, points, 5, (255, 255, 0), -1)
    xianShi(imgcopy1)

# ---------------------------------------------------------------------------------------
    # keep = PD_shape(result, imgcopy_red_thin, 45)
    # for cnt in result[keep]:
    #     P = cv2.boxPoints(cnt)
    #     P = np.int0(P)
    #     imgcopy = cv2.drawContours(imgcopy, [P], -1, (255, 0, 0), 2)
    # xianShi(imgcopy)
# ----------------------------------------------------------------------------------------
    # target_point, target_angle = retrieve_coordinate(
    #     keep, result, 6, imgcopy_red_thin)
    # for i in range(len(target_point)):
    #     points = target_point[i]
    #     points = np.int0(points)
    #     imgcopy = cv2.circle(imgcopy, points, 5, (255, 255, 0), -1)
    # xianShi(imgcopy)

    result_final.append(result_temp_yellow)
    result_final.append(result_temp_purple)
    result_final.append(result_temp_blue)
    result_final.append(result_temp_green)
    result_final.append(result_temp_brown)
    result_final.append(result_temp_orange)
    result_final.append(result_temp_red)
    
    
    
    
    
    
    print(result_final)
    print(len(result_final))
    #run rosnode
    cnt=0
    
    def handler(req):
    	global cnt
    	j=cnt%5
    	i=cnt//5
    	cnt+=1
    	x=int(result_final[i][j][0][0])
    	y=int(result_final[i][j][0][1])
    	angle=result_final[i][j][1]
    	return  cv_pointResponse(x, y, angle)
    
    rospy.init_node('cv_points_node')
    s=rospy.Service('cv_points',cv_point,handler)
    rospy.loginfo('infomation is broadcast')
    rospy.spin()
    
    
    
