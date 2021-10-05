import csv
import matplotlib.pyplot as plt
x = []
y = []
s_x = []
s_y = []
r_x = []
r_y = []
rt= []
ct = []
f = open('data4.txt','r')
i = 0
while True: #True
	line = f.readline()
	if not line: break
	if i>0:
		if line[0] == 'x':
			x.append(float(line[3:]))
		elif line[0] == 'y':
			y.append(float(line[3:]))
		elif 'r_x' in line:
			r_x.append(float(line[5:]))
		elif 'r_y' in line:
			r_y.append(float(line[5:]))
		elif 's_x' in line:
			s_x.append(float(line[5:]))
		elif 's_y' in line:
			s_y.append(float(line[5:]))
		elif 'rt' in line:
			rt.append(float(line[4:]))
		elif 'ct' in line:
			ct.append(float(line[4:]))
	i +=1
f.close()
#plt.plot(x,y)
#plt.show()
plt.plot(s_x,s_y,'o')
plt.show()
#plt.plot(rt,ct)
#plt.show()
