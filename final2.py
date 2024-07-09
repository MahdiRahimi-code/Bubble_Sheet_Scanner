import cv2
import numpy as np
import pytesseract
from PIL import Image
import openpyxl


def calculate_distance(point1, point2):
    point1 = np.array(point1)
    point2 = np.array(point2)
    distance = np.linalg.norm(point1 - point2)
    return distance


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

        if distance < min_bubble_dis:
            min_bubble_dis = distance
            min_bubble_dis_index = bubble[1]
            min_cx=bubble_cx
            min_cy=bubble_cy
    
    min_bubble = ((min_cx, min_cy), min_bubble_dis_index)

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

        if distance < min_bubble_dis:
            min_bubble_dis = distance
            min_bubble_dis_index = bubble[1]
            min_cx=bubble_cx
            min_cy=bubble_cy
    
    min_bubble = ((min_cx, min_cy), min_bubble_dis_index)

    # cv2.drawContours(contour_image, [contours[min_bubble_dis_index]], -1, (240, 42, 12), 3)

    return min_bubble


def awnsers_match(awnsered_letter, awnsers_list):
    global student_awnser_bubbles_sorted, left_bottom_awnser_bubbles_sorted

    for i in awnsers_list:
        nearest_bubble = find_bottom_left_nearest_bubble(i[1])
        Question = left_bottom_awnser_bubbles_sorted.index(nearest_bubble)+1
        if i[0][0] > 400:
            Question+=50
        if i[0][0] > 750:
            Question+=50
        student_awnsers[Question] = awnsered_letter
        student_awnsers_and_locations[(Question, i[0])]=awnsered_letter


# Load and preprocess the image
image = cv2.imread('C:\\Users\\AceR\\Desktop\\CamScanner 06-11-2024 20.32_07.jpg')
gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
adaptive_thresh_gaussian = cv2.adaptiveThreshold(gray_image, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 57, 4)
inverted_image = cv2.bitwise_not(adaptive_thresh_gaussian)
laplacian = cv2.Laplacian(inverted_image, cv2.CV_64F)
laplacian_abs = cv2.convertScaleAbs(laplacian)
new = cv2.bitwise_xor(laplacian_abs, inverted_image)
kernel = np.ones((5, 5), np.uint8)
erosion = cv2.erode(new, kernel, iterations=1)
contours, hierarchy = cv2.findContours(erosion, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
contour_image = np.zeros_like(image)

for contour in contours:
    area = cv2.contourArea(contour)
    if 600 < area < 1300:
        cv2.drawContours(contour_image, [contour], -1, (255, 255, 255), thickness=cv2.FILLED)


contour_image = cv2.cvtColor(contour_image,cv2.COLOR_BGR2GRAY)


# Find the coordinates of the contours and categorize them based on x-coordinates
height, width = contour_image.shape
print(height, width)



left_top_awnser_bubbles = []
right_top_awnser_bubbles = []
left_bottom_awnser_bubbles = []
right_bottom_awnser_bubbles = []
student_id_bubbles = []
student_awnsers = {}
student_id_dict = {}
student_awnsers_and_locations = {}
true_student_awnsers = []
false_student_awnsers = []
# 0:column1 / 1 : column2 , ....
student_awnser_bubbles = [[], [], [], [], [], [], [], [], [], [], [], []]
student_awnser_bubbles_sorted = [[], [], [], [], [], [], [], [], [], [], [], []]


true_awnsers = {}
for i in range(0, 150):
    if i%4==0:
        true_awnsers[i+1] = 'A'
    elif i%4==1:
        true_awnsers[i+1] = 'B'
    elif i%4==2:
        true_awnsers[i+1] = 'C'
    elif i%4==3:
        true_awnsers[i+1] = 'D'



contour_image = cv2.cvtColor(contour_image,cv2.COLOR_GRAY2BGR)


for i, contour in enumerate(contours):
    area = cv2.contourArea(contour)
    M = cv2.moments(contour)
    if M['m00'] != 0:
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        if 600 < area < 1300 and cx < 100 and 700 < cy <3100 :
            left_bottom_awnser_bubbles.append(((cx, cy), i))
            # cv2.drawContours(contour_image, [contour], -1, (240, 42, 12), 3)
        elif 600 < area < 1300 and cx > 2000 and 700 < cy <3100:
            right_bottom_awnser_bubbles.append(((cx, cy),i))
            # cv2.drawContours(contour_image, [contour], -1, (240, 42, 12), 3)
        elif 600 < area < 1300 and 100 < cx < 2000 and 700 < cy <3100:
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
                # print(cx, cy)
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
                # cv2.drawContours(contour_image, [contour], -1, (0, 255, 0), 3)
            

        elif 600 < area < 1300 and 800 < cx < 2000 and cy < 700:
            student_id_bubbles.append(((cx, cy), i))
            # cv2.drawContours(contour_image, [contour], -1, (0, 255, 0), 3)
        elif 600 < area < 1300 and cx < 100 and cy < 700:
            # cv2.drawContours(contour_image, [contour], -1, (240, 42, 12), 3)
            left_top_awnser_bubbles.append(((cx, cy),i))
        elif 600 < area < 1300 and cx > 2000 and cy < 700:
            # cv2.drawContours(contour_image, [contour], -1, (240, 42, 12), 3)
            right_top_awnser_bubbles.append(((cx, cy), i))



#sorting bubbles
student_id_bubbles_sorted = sorted(student_id_bubbles, key=lambda bubble: bubble[0][0])
right_top_awnser_bubbles_sorted = sorted(right_top_awnser_bubbles, key=lambda bubble: bubble[0][1])
left_top_awnser_bubbles_sorted = sorted(right_top_awnser_bubbles, key=lambda bubble: bubble[0][1])
left_bottom_awnser_bubbles_sorted = sorted(left_bottom_awnser_bubbles, key=lambda bubble: bubble[0][1])
for i in range(len(student_awnser_bubbles)):
    student_awnser_bubbles_sorted[i] = sorted(student_awnser_bubbles[i], key=lambda bubble: bubble[0][1])


# finding Student ID
student_id = ''
for i in student_id_bubbles_sorted:
    nearest_bubble = find_top_right_nearest_bubble(i[1])
    number = right_top_awnser_bubbles_sorted.index(nearest_bubble)
    student_id+=str(number)


# print(f"\t student_id : {student_id}")



for i in range(len(student_awnser_bubbles_sorted)):
    if i%4==0:
        awnsers_match('A', student_awnser_bubbles_sorted[i])
    elif i%4==1:
        awnsers_match('B', student_awnser_bubbles_sorted[i])
    elif i%4==2:
        awnsers_match('C', student_awnser_bubbles_sorted[i])
    if i%4==3:
        awnsers_match('D', student_awnser_bubbles_sorted[i])


# sorting awnsers
student_awnsers = dict(sorted(student_awnsers.items()))
student_awnsers_and_locations = dict(sorted(student_awnsers_and_locations.items()))


# checking true or false awnsers
for awnser in student_awnsers_and_locations.keys():
    if student_awnsers[awnser[0]]==true_awnsers[awnser[0]]:
        true_student_awnsers.append(awnser)
    else:
        false_student_awnsers.append(awnser)


# highliting True and False awnsers on the original image
for i in true_student_awnsers:
    cv2.circle(image, i[1], 20, (32, 212, 8), 4)
for i in false_student_awnsers:
    cv2.circle(image, i[1], 20, (0, 0, 255), 4)

score = len(true_student_awnsers)




# Creating an excel file
file_path = 'C:\\Users\\AceR\\Desktop\\BubbleSheet.xlsx'
try:
    workbook = openpyxl.load_workbook(file_path)
    print("Existing workbook loaded.")
except FileNotFoundError:
    workbook = openpyxl.Workbook()
    print("New workbook created.")


sheet = workbook.active
sheet.title = "Sheet1"


sheet['A1'] = 'Question'
sheet['B1'] = 'Student Awnser'
sheet['C1'] = 'True Awnser'
sheet['D1'] = 'Correct'


t=2
for i in true_awnsers.keys():
    sheet['A'+str(t)] = str(i)
    if i in student_awnsers.keys():  #student filled
        found = False
        for j in true_student_awnsers:
            if i==j[0]:
                found=True
                break
        sheet['B'+str(t)] = student_awnsers[i]
        sheet['C'+str(t)] = true_awnsers[i]
        if found:   # awnsered True
            sheet['D'+str(t)] = 'True'
        else:   # awnsered False
            sheet['D'+str(t)] = 'False'
    else:
        sheet['B'+str(t)] = '-'
        sheet['C'+str(t)] = str(true_awnsers[i])
        sheet['D'+str(t)] = '-'
    t+=1


# saving in an excel file
workbook.save(file_path)
print(f"Workbook saved at {file_path}")


resized_org = cv2.resize(image, (800, 600), interpolation=cv2.INTER_CUBIC)
resized_org2 = cv2.resize(contour_image, (800, 600), interpolation=cv2.INTER_CUBIC)
cv2.imshow('original', resized_org)
cv2.imshow('contooyr', resized_org2)
cv2.waitKey(0)