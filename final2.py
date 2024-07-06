import cv2
import numpy as np
import pytesseract
from PIL import Image



# Load and preprocess the image
image = cv2.imread('C:\\Users\\AceR\\Desktop\\CamScanner 06-11-2024 20.32_08.jpg')
kham = cv2.imread('C:\\Users\\AceR\\Desktop\\AwnserSheet.jpg')
gray_kham= cv2.cvtColor(kham, cv2.COLOR_BGR2GRAY)
gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
_, thresh = cv2.threshold(gray_image, 80, 255, cv2.THRESH_BINARY_INV)
adaptive_thresh_gaussian = cv2.adaptiveThreshold(gray_image, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 57, 4)
inverted_image = cv2.bitwise_not(adaptive_thresh_gaussian)
# اعمال فیلتر لاپلاسین
laplacian = cv2.Laplacian(inverted_image, cv2.CV_64F)
# تبدیل مقادیر منفی به مقادیر مثبت
laplacian_abs = cv2.convertScaleAbs(laplacian)



new = cv2.bitwise_xor(laplacian_abs, inverted_image)

# ساختن کرنل مورفولوژیکال
kernel = np.ones((5, 5), np.uint8)

# اعمال فرسایش (Erosion)
erosion = cv2.erode(new, kernel, iterations=1)

contours, hierarchy = cv2.findContours(erosion, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# ایجاد ماسک برای رسم کانتورهای با مساحت بین 1000 تا 2000 پیکسل
contour_image = np.zeros_like(image)

for contour in contours:
    area = cv2.contourArea(contour)
    if 600 < area < 1300:
        cv2.drawContours(contour_image, [contour], -1, (255, 255, 255), thickness=cv2.FILLED)


contour_image = cv2.cvtColor(contour_image,cv2.COLOR_BGR2GRAY)



# Find the coordinates of the contours and categorize them based on x-coordinates
height, width = contour_image.shape
print(height, width)


def calculate_distance(point1, point2):
    point1 = np.array(point1)
    point2 = np.array(point2)
    distance = np.linalg.norm(point1 - point2)
    return distance

left_top_awnser_bubbles = []
right_top_awnser_bubbles = []
left_bottom_awnser_bubbles = []
right_bottom_awnser_bubbles = []
student_id_bubbles = []
awnsers = {}
student_id_dict = {}
# 0:column1 / 1 : column2 , ....
student_awnser_bubbles = [[], [], [], [], [], [], [], [], [], [], [], []]
student_awnser_bubbles_sorted = [[], [], [], [], [], [], [], [], [], [], [], []]



contour_image = cv2.cvtColor(contour_image,cv2.COLOR_GRAY2BGR)

min_y = float('inf')
min_y_index = -1


max_y = -1
max_y_index = -1

for i, contour in enumerate(contours):
    area = cv2.contourArea(contour)
    M = cv2.moments(contour)
    if M['m00'] != 0:
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        if 600 < area < 1300 and cx < 100 and cy > 700:
            left_bottom_awnser_bubbles.append(((cx, cy), i))
            # cv2.drawContours(contour_image, [contour], -1, (240, 42, 12), 3)
            if cy < min_y:
                min_y = cy
                min_y_index = i
        elif 600 < area < 1300 and cx > 2000 and cy > 700:
            right_bottom_awnser_bubbles.append(((cx, cy),i))
            # cv2.drawContours(contour_image, [contour], -1, (240, 42, 12), 3)
            if cy > max_y:
                max_y = cy
                max_y_index = i
        elif 600 < area < 1300 and 100 < cx < 2000 and cy > 700:
            if cx<200:
                # cv2.drawContours(contour_image, [contour], -1, (0, 255, 0), 3)
                student_awnser_bubbles[0].append(((cx, cy), i))
            elif 200 < cx < 250:
                student_awnser_bubbles[1].append(((cx, cy), i))
                # cv2.drawContours(contour_image, [contour], -1, (0, 255, 0), 3)
            elif 250 < cx < 300:
                student_awnser_bubbles[2].append(((cx, cy), i))
                # cv2.drawContours(contour_image, [contour], -1, (0, 255, 0), 3)
            elif 300 < cx < 400:
                student_awnser_bubbles[3].append(((cx, cy), i))
                # cv2.drawContours(contour_image, [contour], -1, (0, 255, 0), 3)
            elif 400 < cx < 500:
                student_awnser_bubbles[4].append(((cx, cy), i))
                # cv2.drawContours(contour_image, [contour], -1, (0, 255, 0), 3)
            elif 500 < cx < 550:
                student_awnser_bubbles[5].append(((cx, cy), i))
                # cv2.drawContours(contour_image, [contour], -1, (0, 255, 0), 3)
            elif 550 < cx < 650:
                student_awnser_bubbles[6].append(((cx, cy), i))
                # cv2.drawContours(contour_image, [contour], -1, (0, 255, 0), 3)
            elif 650 < cx < 750:
                student_awnser_bubbles[7].append(((cx, cy), i))
                # cv2.drawContours(contour_image, [contour], -1, (0, 255, 0), 3)
            elif 750 < cx < 850:
                student_awnser_bubbles[8].append(((cx, cy), i))
                # cv2.drawContours(contour_image, [contour], -1, (0, 255, 0), 3)
            elif 850 < cx < 900:
                student_awnser_bubbles[9].append(((cx, cy), i))
                # cv2.drawContours(contour_image, [contour], -1, (0, 255, 0), 3)
            elif 900 < cx < 950:
                student_awnser_bubbles[10].append(((cx, cy), i))
                # cv2.drawContours(contour_image, [contour], -1, (0, 255, 0), 3)
            elif 950 < cx < 1000:
                student_awnser_bubbles[11].append(((cx, cy), i))
                cv2.drawContours(contour_image, [contour], -1, (0, 255, 0), 3)
            

        elif 600 < area < 1300 and cx < 2000 and cx>300 and cy < 700:
            student_id_bubbles.append(((cx, cy), i))
            # cv2.drawContours(contour_image, [contour], -1, (0, 255, 0), 3)
        elif 600 < area < 1300 and cx < 100 and cy < 700:
            # cv2.drawContours(contour_image, [contour], -1, (240, 42, 12), 3)
            left_top_awnser_bubbles.append(((cx, cy),i))
        elif 600 < area < 1300 and cx > 2000 and cy < 700:
            # cv2.drawContours(contour_image, [contour], -1, (240, 42, 12), 3)
            right_top_awnser_bubbles.append(((cx, cy), i))

if min_y_index != -1:
    cv2.drawContours(contour_image, [contours[min_y_index]], -1, (0, 255, 0), 3)
if max_y_index != -1:
    cv2.drawContours(contour_image, [contours[max_y_index]], -1, (0, 255, 0), 3)


#sorting bubbles
student_id_bubbles_sorted = sorted(student_id_bubbles, key=lambda bubble: bubble[0][0])
right_top_awnser_bubbles_sorted = sorted(right_top_awnser_bubbles, key=lambda bubble: bubble[0][1])
left_top_awnser_bubbles_sorted = sorted(right_top_awnser_bubbles, key=lambda bubble: bubble[0][1])


left_bottom_awnser_bubbles_sorted = sorted(left_bottom_awnser_bubbles, key=lambda bubble: bubble[0][1])

for i in range(len(student_awnser_bubbles[0])):
    student_awnser_bubbles_sorted[i] = sorted(student_awnser_bubbles[i], key=lambda bubble: bubble[0][1])



# for i in left_top_awnser_bubbles:
#     print(min_bubble_dis)
#     print(calculate_distance((i[0][0], i[0][1]), (cx, cy)))
#     if calculate_distance((i[0][0], i[0][1]), (cx, cy))<min_bubble_dis:
#         min_bubble_dis = calculate_distance(i[0][0], i[0][1])
#         min_bubble_dis_index = i[1]


def find_top_right_nearest_bubble(contour_id):
    global right_top_awnser_bubbles_sorted
    
    min_bubble_dis = float('inf')
    min_bubble_dis_index = -1
    min_cx = -1
    min_cy = -1

    M = cv2.moments(contours[contour_id])
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])

    # cv2.drawContours(contour_image, [contours[contour_id]], -1, (240, 42, 12), 3)

    for bubble in right_top_awnser_bubbles_sorted:
        bubble_cx, bubble_cy = bubble[0]
        distance = calculate_distance((bubble_cx, bubble_cy), (cx, cy))
        # print(f"Current minimum distance: {min_bubble_dis}")
        # print(f"Calculated distance: {distance}")

        if distance < min_bubble_dis:
            min_bubble_dis = distance
            min_bubble_dis_index = bubble[1]
            min_cx=bubble_cx
            min_cy=bubble_cy
    
    min_bubble = ((min_cx, min_cy), min_bubble_dis_index)
    # print(min_bubble)

    # cv2.drawContours(contour_image, [contours[min_bubble_dis_index]], -1, (240, 42, 12), 3)

    return min_bubble


def find_bottom_left_nearest_bubble(contour_id):
    global left_bottom_awnser_bubbles_sorted
    
    min_bubble_dis = float('inf')
    min_bubble_dis_index = -1
    min_cx = -1
    min_cy = -1

    M = cv2.moments(contours[contour_id])
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])

    # cv2.drawContours(contour_image, [contours[contour_id]], -1, (240, 42, 12), 3)

    for bubble in left_bottom_awnser_bubbles_sorted:
        bubble_cx, bubble_cy = bubble[0]
        distance = calculate_distance((bubble_cx, bubble_cy), (cx, cy))
        # print(f"Current minimum distance: {min_bubble_dis}")
        # print(f"Calculated distance: {distance}")

        if distance < min_bubble_dis:
            min_bubble_dis = distance
            min_bubble_dis_index = bubble[1]
            min_cx=bubble_cx
            min_cy=bubble_cy
    
    min_bubble = ((min_cx, min_cy), min_bubble_dis_index)
    # print(min_bubble)

    # cv2.drawContours(contour_image, [contours[min_bubble_dis_index]], -1, (240, 42, 12), 3)

    return min_bubble


student_id = ''

for i in student_id_bubbles_sorted:
    print("i: ",i[1])
    nearest_bubble = find_top_right_nearest_bubble(i[1])
    print(nearest_bubble)
    number = right_top_awnser_bubbles_sorted.index(nearest_bubble)+1
    student_id+=str(number)


print(f"\t student_id : {student_id}")


print()
print(left_bottom_awnser_bubbles_sorted)
print()





def awnsers_match(awnsered_letter, awnsers_list):
    global student_awnser_bubbles_sorted, left_bottom_awnser_bubbles_sorted

    for i in awnsers_list:
        # print("i2: ",i[1])
        nearest_bubble = find_bottom_left_nearest_bubble(i[1])
        # print(nearest_bubble)
        Question = left_bottom_awnser_bubbles_sorted.index(nearest_bubble)+1
        if (i in student_awnser_bubbles_sorted[4]) or (i in student_awnser_bubbles_sorted[5]) or (i in student_awnser_bubbles_sorted[6]) or (i in student_awnser_bubbles_sorted[7]):
            Question+=50
        if (i in student_awnser_bubbles_sorted[8]) or (i in student_awnser_bubbles_sorted[9]) or (i in student_awnser_bubbles_sorted[10]) or (i in student_awnser_bubbles_sorted[11]):
            Question+=50
        awnsers[Question] = awnsered_letter


for i in range(len(student_awnser_bubbles_sorted)):
    if i%4==0:
        awnsers_match('A', student_awnser_bubbles_sorted[i])
    elif i%4==1:
        awnsers_match('B', student_awnser_bubbles_sorted[i])
    elif i%4==2:
        awnsers_match('C', student_awnser_bubbles_sorted[i])
    if i%4==3:
        awnsers_match('D', student_awnser_bubbles_sorted[i])

print(awnsers)


######################################################################################



minM = cv2.moments(contours[min_y_index])
maxM = cv2.moments(contours[max_y_index])
mincx = int(minM['m10'] / minM['m00'])-20
mincy = int(minM['m01'] / minM['m00'])-20
maxcx = int(maxM['m10'] / maxM['m00'])+20
maxcy = int(maxM['m01'] / maxM['m00'])+20
cropped = contour_image[mincy:maxcy, mincx:maxcx]
resizedc = cv2.resize(cropped, (800, 600), interpolation=cv2.INTER_CUBIC)
cv2.imshow('Cropped', resizedc)

resized = cv2.resize(contour_image, (800, 600), interpolation=cv2.INTER_CUBIC)

cv2.imshow('threah', resized)
cv2.imwrite('C:\\Users\\Acer\\Desktop\\2.jpg', contour_image)
cv2.waitKey(0)
cv2.destroyAllWindows()

print(left_bottom_awnser_bubbles)
print(right_bottom_awnser_bubbles)
print()
print(left_top_awnser_bubbles_sorted)

print(student_id_bubbles_sorted)
print(right_top_awnser_bubbles_sorted)



##########################################################################################




# Load and preprocess the image
image_awnser = cv2.imread('C:\\Users\\AceR\\Desktop\\CamScanner 06-11-2024 20.32_07 - Copy.jpg')

gray_image_awnser = cv2.cvtColor(image_awnser, cv2.COLOR_BGR2GRAY)
_, thresh_awnser = cv2.threshold(gray_image_awnser, 80, 255, cv2.THRESH_BINARY_INV)
adaptive_thresh_gaussian_awnser = cv2.adaptiveThreshold(gray_image_awnser, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 57, 4)
inverted_image_awnser = cv2.bitwise_not(adaptive_thresh_gaussian_awnser)
# اعمال فیلتر لاپلاسین
laplacian_awnser = cv2.Laplacian(inverted_image_awnser, cv2.CV_64F)
# تبدیل مقادیر منفی به مقادیر مثبت
laplacian_abs_awnser = cv2.convertScaleAbs(laplacian_awnser)



new_awnser = cv2.bitwise_xor(laplacian_abs_awnser, inverted_image_awnser)

# ساختن کرنل مورفولوژیکال
kernel_awnser = np.ones((5, 5), np.uint8)

# اعمال فرسایش (Erosion)
erosion_awnser = cv2.erode(new_awnser, kernel_awnser, iterations=1)

contours_a, hierarchy_a = cv2.findContours(erosion_awnser, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# ایجاد ماسک برای رسم کانتورهای با مساحت بین 1000 تا 2000 پیکسل
contour_image_a = np.zeros_like(image_awnser)

for contour in contours_a:
    area = cv2.contourArea(contour)
    if 600 < area < 1300:
        cv2.drawContours(contour_image_a, [contour], -1, (255, 255, 255), thickness=cv2.FILLED)


#contour_image_a = cv2.cvtColor(contour_image_a ,cv2.COLOR_GRAY2BGR)


max_y_a = -1
max_y_index_a = -1
min_y_a = float('inf')
min_y_index_a = -1

left_awnser_bubbles_a = []
right_awnser_bubbles_a = []

for i, contour in enumerate(contours_a):
    area = cv2.contourArea(contour)
    M = cv2.moments(contour)
    if M['m00'] != 0:
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        if 600 < area < 1300 and cx < 100 and cy > 700:
            left_awnser_bubbles_a.append((cx, cy))
            if cy < min_y_a:
                min_y_a = cy
                min_y_index_a = i
        elif 600 < area < 1300 and cx > 2000 and cy > 700:
            right_awnser_bubbles_a.append((cx, cy))
            if cy > max_y_a:
                max_y_a = cy
                max_y_index_a = i

if min_y_index_a != -1:
    cv2.drawContours(contour_image_a, [contours_a[min_y_index_a]], -1, (0, 255, 0), 3)
if max_y_index_a != -1:
    cv2.drawContours(contour_image_a, [contours_a[max_y_index_a]], -1, (0, 255, 0), 3)

minM_a = cv2.moments(contours_a[min_y_index_a])
maxM_a = cv2.moments(contours_a[max_y_index_a])
mincx_a = int(minM_a['m10'] / minM_a['m00'])-20
mincy_a = int(minM_a['m01'] / minM_a['m00'])-20
maxcx_a = int(maxM_a['m10'] / maxM_a['m00'])+20
maxcy_a = int(maxM_a['m01'] / maxM_a['m00'])+20
cropped_a = contour_image_a[mincy_a:maxcy_a, mincx_a:maxcx_a]
resizedc_a = cv2.resize(cropped_a, (800, 600), interpolation=cv2.INTER_CUBIC)
cv2.imshow('Cropped', resizedc_a)

height1, width1 = cropped.shape[:2]
height2, width2 = cropped_a.shape[:2]

# تغییر اندازه‌ی تصویر دوم به اندازه‌ی تصویر اول (در صورت نیاز)
if (height1 != height2) or (width1 != width2):
    cropped_a = cv2.resize(cropped_a, (width1, height1))

# ترکیب تصاویر با استفاده از عملگر AND
result = cv2.bitwise_and(cropped, cropped_a)


resized_a = cv2.resize(result, (800, 600), interpolation=cv2.INTER_CUBIC)
cv2.imshow('threah', resized_a)
cv2.waitKey(0)
cv2.destroyAllWindows()