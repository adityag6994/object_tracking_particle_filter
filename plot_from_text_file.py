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
x_lpf = []
y_lpf = []
z_lpf = []
roll_lpf  = []
pitch_lpf = []
yaw_lpf   = []


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
        z.append(float(row[2]))
        x_lpf.append(float(row[6]))
        y_lpf.append(float(row[7]))
        z_lpf.append(float(row[8]))
        roll.append((float(row[3])))
        pitch.append((float(row[4])))
        yaw.append(float(row[5]))
        roll_lpf.append((float(row[9])))
        pitch_lpf.append((float(row[10])))
        yaw_lpf.append(float(row[11]))     
        # if(count == 300):
            # break;
        # roll.append((float(row[3]))*180/pi )
        # pitch.append((float(row[4]))*180/pi)
        # yaw.append(float(row[5])*180/pi)
        # roll_lpf.append((float(row[3]))*180/pi )
        # pitch_lpf.append((float(row[4]))*180/pi)
        # yaw_lpf.append(float(row[5])*180/pi)     

k = []
current_roll = []
count_reference = 0
with open(sys.argv[2],'r') as csvfile:
    plots = csv.reader(csvfile, delimiter=' ')
    for row in plots:
        k.append(count_reference)
        count_reference = count_reference + 1
        current_roll.append((float(row[0]))-95)
        # current_angle.append(((float(row[0]) - 65)*90/80)-45)

mean_x = sum(x)/count
mean_y = sum(y)/count
mean_z = sum(z)/count
mean_roll  = sum(roll)/count
mean_pitch = sum(pitch)/count
mean_yaw   = sum(yaw)/count
mean_x_lpf = sum(x_lpf)/count 
mean_y_lpf = sum(y_lpf)/count 
mean_z_lpf = sum(z_lpf)/count 
mean_roll_lpf  = sum(roll_lpf)/count
mean_pitch_lpf = sum(pitch_lpf)/count
mean_yaw_lpf   = sum(yaw_lpf)/count


print('Mean x : ', mean_x)
print('Mean y : ', mean_y)
print('Mean z : ', mean_z)
print('Mean roll  : ', mean_roll)
print('Mean pitch : ', mean_pitch)
print('Mean yaw   : ', mean_yaw)
print('Mean x_lpf : ', mean_x_lpf)
print('Mean y_lpf : ', mean_y_lpf)
print('Mean z_lpf : ', mean_z_lpf)
print('Mean roll_lpf  : ', mean_roll_lpf)
print('Mean pitch_lpf : ', mean_pitch_lpf)
print('Mean yaw_lpf   : ', mean_yaw_lpf)

f, (ax1, ax2, ax3, ax4, ax5, ax6) = plt.subplots(6, sharex=True, sharey=True)
l1,=ax1.plot(i,roll, color='r', label='Blue stars')
l2,=ax2.plot(i,pitch, color='g')
l3,=ax3.plot(i,yaw, color='b')
lr,=ax3.plot(k,current_roll, color='r')
l4,=ax4.plot(i,roll_lpf, color='r', label='Blue stars')
l5,=ax5.plot(i,pitch_lpf, color='g')
l6,=ax6.plot(i,yaw_lpf, color='b')
lr,=ax6.plot(k,current_roll, color='r')

ax1.set_title('Eurler Angles (Degre) vs Count (5 fps)')
plt.legend([l1, l2, l3, l4, l5, l6],["Pitch", "Yaw", "Roll","PitchLPF", "YawLPF", "RollLPF"])
ax1.axhline(y=mean_roll,c="y",linewidth=1.5,zorder=0)
ax2.axhline(y=mean_pitch,c="y",linewidth=1.5,zorder=0)
ax3.axhline(y=mean_yaw,c="y",linewidth=1.5,zorder=0)
ax4.axhline(y=mean_roll_lpf,c="y",linewidth=1.5,zorder=0)
ax5.axhline(y=mean_pitch_lpf,c="y",linewidth=1.5,zorder=0)
ax6.axhline(y=mean_yaw_lpf,c="y",linewidth=1.5,zorder=0)
plt.show()

exit()
f, (ax1, ax2, ax3, ax4, ax5, ax6) = plt.subplots(6, sharex=True, sharey=True)
l1,=ax1.plot(i,x, color='r', label='Blue stars')
l2,=ax2.plot(i,y, color='g')
l3,=ax3.plot(i,z, color='b')
l4,=ax4.plot(i,x_lpf, color='r', label='Blue stars')
l5,=ax5.plot(i,y_lpf, color='g')
l6,=ax6.plot(i,z_lpf, color='b')
ax1.set_title('Positio (m) vs Count (5 fps)')
plt.legend([l1, l2, l3,l4,l5,l6],["x", "y", "z","xLPF", "yLPF", "zLPF"])
ax1.axhline(y=mean_x,c="y",linewidth=1.5,zorder=0)
ax2.axhline(y=mean_y,c="y",linewidth=1.5,zorder=0)
ax3.axhline(y=mean_z,c="y",linewidth=1.5,zorder=0)
ax4.axhline(y=mean_x_lpf,c="y",linewidth=1.5,zorder=0)
ax5.axhline(y=mean_y_lpf,c="y",linewidth=1.5,zorder=0)
ax6.axhline(y=mean_z_lpf,c="y",linewidth=1.5,zorder=0)
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
