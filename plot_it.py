import matplotlib.pyplot as plt
import csv
import sys

i = [] 
current_angle = []

pi = 3.14
count = 0

# with open('/home/aditya/catkin_ws/log.txt','r') as csvfile:
with open(sys.argv[1],'r') as csvfile:
    plots = csv.reader(csvfile, delimiter=' ')
    for row in plots:
        # print('count => ',count, ' ' ,row)
        i.append(count)
        count = count + 1
        current_angle.append(((float(row[0]))))
        # current_angle.append(((float(row[0]) - 65)*90/80)-45)


f, (ax1) = plt.subplots(1, sharex=True, sharey=True)
l1,=ax1.plot(i,current_angle, color='r', label='Blue stars')
ax1.set_title('Eurler Angles (Degre) vs Count (5 fps)')
plt.legend([l1],["current able"])
print(current_angle)
plt.show()

power_smooth = spline()
# 65  : -45
# 140 :  45