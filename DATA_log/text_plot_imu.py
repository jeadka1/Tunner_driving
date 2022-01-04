import matplotlib.pyplot as plt

path = 'C:/Users/owner/Desktop/Workspace_paper/temp/trash/cjs/'
 

#f = open(path + "pose_data.txt", 'r')
f = open('data4.txt','r')

lines = f.readlines()
 

x = []
y = []
z = []
 

for idx, line in enumerate(lines):
    if 'position:' in line:
        x_line = lines[idx + 1].split(" ")
        y_line = lines[idx + 2].split(" ")
        # z_line = lines[idx + 3].split(" ")
        if 'x:' in x_line:
            x.append(float(x_line[x_line.index("x:")+1]))
        if 'y:' in y_line:
            y.append(float(y_line[y_line.index("y:")+1]))
        # if 'z:' in z_line:
        #     z.append(float(z_line[z_line.index("z:")+1]))
    elif 'orientation:' in line:
        # x_line = lines[idx + 1].split(" ")
        # y_line = lines[idx + 2].split(" ")
        z_line = lines[idx + 3].split(" ")
        if 'z:' in z_line:
            z.append(float(z_line[z_line.index("z:")+1]))
    else:
        pass

plt.figure()
plt.plot(x)

plt.figure()
plt.plot(y)

plt.figure()
plt.plot(z)

plt.show()
