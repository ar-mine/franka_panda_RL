import numpy as np
import cv2
import math


class BoxDetector:
    def __init__(self):
        pass

    def gamma_trans(self,img,gamma):
        gamma_table=[np.power(x/255.0,gamma)*255.0 for x in range(256)]
        gamma_table=np.round(np.array(gamma_table).astype(np.uint8))
        return  cv2.LUT(img,gamma_table)

    def process(self, img_):

        img_=self.gamma_trans(img_,2.0)
        # read image
        img_gray = cv2.cvtColor(img_, cv2.COLOR_BGR2GRAY)

        # img = cv2.GaussianBlur(img,(3,3),0)
        edges = cv2.Canny(img_gray, 50, 150, apertureSize=3)

        # kernel = np.ones((1,1),np.uint8)
        # edges = cv2.dilate(edges,kernel,iterations = 1)
        # edges = cv2.erode(edges,kernel,iterations = 1)
        # cv2.imshow('canny',edges)
        # cv2.waitKey()

        lines = cv2.HoughLines(edges, 2, 2 * np.pi / 180, 150)  # 这里对最后一个参数使用了经验型的值
        result = img_.copy()

        line_h = []
        line_v = []
        if lines is not None:
            for line in lines:
                rho = line[0][0]  # 第一个元素是距离rho
                theta = line[0][1]  # 第二个元素是角度theta

                if (theta < (np.pi / 4.)) or (theta > (6. * np.pi / 8.0)):  # 垂直直线
                    pt1 = [int(rho / np.cos(theta)), 0]  # 该直线与第一行的交点
                    # 该直线与最后一行的焦点
                    pt2 = [int((rho - result.shape[0] * np.sin(theta)) / np.cos(theta)), result.shape[0]]
                    # cv2.line( result, pt1, pt2, (255,255,255), 1)             # 绘制一条白线
                    line_v.append([pt1, pt2])
                elif (theta > (2. * np.pi / 8.)) or (theta < (6. * np.pi / 8.0)):  # 水平直线
                    pt1 = [0, int(rho / np.sin(theta))]  # 该直线与第一列的交点
                    # 该直线与最后一列的交点
                    pt2 = [result.shape[1], int((rho - result.shape[1] * np.cos(theta)) / np.sin(theta))]
                    # cv2.line(result, pt1, pt2, (255,255,255), 1)
                    line_h.append([pt1, pt2])

            # combine lines
            line_v = comb_lines(line_v, 0)
            line_h = comb_lines(line_h, 1)

            #refine lines
            line_v = check_lines(line_v, 0)
            line_h = check_lines(line_h, 1)

            '''
            # draw lines
            for line in line_v:
                cv2.line(result, line[0], line[1], (0,255,0))
            for line in line_h:
                cv2.line(result, line[0], line[1], (255,0,0))
            '''

            # obtain the cross rectangle
            rects = []
            bbox_list = []
            for l_h in range(len(line_h) - 1):

                for l_v in range(len(line_v) - 1):
                    # rect_s = [line_v[l_v][0][0],line_h[l_h][0][1]]
                    # rect_e = [line_v[l_v + 1][0][0],line_h[l_h+1][0][1]]

                    rect_s = cal_cross_point(line_h[l_h], line_v[l_v][0][0], 0)
                    rect_e = cal_cross_point(line_h[l_h + 1], line_v[l_v + 1][0][0], 0)
                    rects.append([rect_s, rect_e])

            print('rectangle number: ', len(rects))
            for rec in rects:
                cv2.rectangle(result, rec[0], rec[1], (0, 255, 0), 2)
                bbox_list.append(rec[0]+rec[1])
        else:
            bbox_list = []
        # image[np.where(dilation>0)]=[0,255,0]
        return result, bbox_list


# 计算两边夹角额cos值
def angle_cos(p0, p1, p2):
    d1, d2 = (p0 - p1).astype('float'), (p2 - p1).astype('float')
    return abs(np.dot(d1, d2) / np.sqrt(np.dot(d1, d1) * np.dot(d2, d2)))


def check_lines(lines, v_h=0):
    if len(lines)<4:
        return lines
    else:
        lines_new=[]
        mid=(lines[0][0][v_h]+lines[-1][0][v_h])/2

        diffs=[]
        for line in lines:
            diffs.append(abs(line[0][v_h]-mid))

        mid_line=lines[diffs.index(min(diffs))]

        return [lines[0],mid_line,lines[-1]]


def comb_lines(lines, v_h=0):
    if v_h==0:
        wh=640
    else:
        wh=480

    # sorting
    ind_b = [x[0][v_h] for x in lines]
    ind_e = [x[1][v_h] for x in lines]

    sorted_id = sorted(range(len(ind_b)), key=lambda k: ind_b[k], reverse=False)

    lines_comb = []

    previous = []
    for i, s_id in enumerate(sorted_id):
        if i == 0:
            previous = lines[s_id]
            continue
        if lines[s_id][0][v_h] - previous[0][v_h] < 30:
            previous[0][v_h] = int((previous[0][v_h] + lines[s_id][0][v_h]) / 2)
            previous[1][v_h] = int((previous[1][v_h] + lines[s_id][1][v_h]) / 2)
        else:
            lines_comb.append(previous)
            previous = lines[s_id]

    lines_comb.append(previous)

    return lines_comb


def cal_cross_point(lines, pos_x=0, pos_y=0):
    if pos_x:
        pos_y = (lines[1][1] - lines[0][1]) * ((pos_x - lines[0][0]) / (lines[1][0] - lines[0][0])) + lines[0][1]

    else:
        pos_x = (lines[1][0] - lines[0][0]) * ((pos_y - lines[0][1]) / (lines[1][1] - lines[0][1])) + lines[0][0]

    return [int(pos_x), int(pos_y)]


if __name__ == "__main__":
    detector = BoxDetector()
    img = cv2.imread("image.png")
    img, rects = detector.process(img)
    cv2.imshow('img', img)
    cv2.waitKey()
