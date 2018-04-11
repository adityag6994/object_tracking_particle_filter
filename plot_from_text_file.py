import matplotlib.pyplot as plt
import csv
import sys

i = [] 
x = []
y = []
z = [] 
roll  = []
pitch = []
yaw   = []
pi = 3.14
count = 0
# with open('/home/aditya/catkin_ws/log.txt','r') as csvfile:
with open(sys.argv[1],'r') as csvfile:
    plots = csv.reader(csvfile, delimiter=' ')
    for row in plots:
    	# print('count => ',count, ' ' ,row)
    	i.append(count)
    	count = count + 1
        x.append(float(row[0]))
        y.append(float(row[1]))
        z.append(float(row[3]))
        # roll.append((float(row[4]))*180/pi)
        # pitch.append((float(row[5]))*180/pi)
        # yaw.append(float(row[6])*180/pi)
        roll.append((float(row[4]))*180/pi )
        pitch.append((float(row[5]))*180/pi)
        yaw.append(float(row[6])*180/pi)
        # roll  = (roll*180)/pi
        # pitch = (pitch*180)/pi
        # yaw   = (yaw*180)/pi

mean_x = sum(x)/count
mean_y = sum(y)/count
mean_z = sum(z)/count
mean_roll  = sum(roll)/count
mean_pitch = sum(pitch)/count
mean_yaw   = sum(yaw)/count

print('Mean x : ', mean_x)
print('Mean y : ', mean_y)
print('Mean z : ', mean_z)
print('Mean roll  : ', mean_roll)
print('Mean pitch : ', mean_pitch)
print('Mean yaw   : ', mean_yaw)

f, (ax1, ax2, ax3) = plt.subplots(3, sharex=True, sharey=True)
l1,=ax1.plot(i,roll, color='r', label='Blue stars')
l2,=ax2.plot(i,pitch, color='g')
l3,=ax3.plot(i,yaw, color='b')
ax1.set_title('Eurler Angles (Degre) vs Count (5 fps)')
plt.legend([l1, l2, l3],["Pitch", "Yaw", "Roll"])
ax1.axhline(y=mean_roll,c="y",linewidth=1.5,zorder=0)
ax2.axhline(y=mean_pitch,c="y",linewidth=1.5,zorder=0)
ax3.axhline(y=mean_yaw,c="y",linewidth=1.5,zorder=0)
plt.show()

f, (ax1, ax2, ax3) = plt.subplots(3, sharex=True, sharey=True)
l1,=ax1.plot(i,x, color='r', label='Blue stars')
l2,=ax2.plot(i,y, color='g')
l3,=ax3.plot(i,z, color='b')
ax1.set_title('Positio (m) vs Count (5 fps)')
plt.legend([l1, l2, l3],["x", "y", "z"])
ax1.axhline(y=mean_x,c="y",linewidth=1.5,zorder=0)
ax2.axhline(y=mean_y,c="y",linewidth=1.5,zorder=0)
ax3.axhline(y=mean_z,c="y",linewidth=1.5,zorder=0)
plt.show()


#for i in range(0,)
#plt.figure(1)
# plt.subplot(321)
# plt.plot(i,x,'r--')
# plt.subplot(322)
# plt.plot(i,y,'r--')
# plt.subplot(323)
# plt.plot(i,z,'r--')
# plt.xlabel('xlabel')
# plt.ylabel('ylabel')
# plt.subplot(311)
# plt.plot(i,roll,'r--', label='roll')
# plt.subplot(312)
# plt.plot(i,pitch,'g--')
# plt.subplot(313)
# plt.plot(i,yaw,'b--')
# plt.show()

# plt.plot(x,y,z,roll,pitch,yaw label='Loaded from file!')
# plt.xlabel('x')
# plt.ylabel('y')
# plt.title('Interesting Graph\nCheck it out')
# plt.legend()
# plt.show()