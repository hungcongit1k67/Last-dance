# import cv2
from PIL import Image

img = Image.open('MHK.png')
img.load()
grid = []
for i in range(80):
    row = []
    for j in range(80):
        x = int(i * img.size[0] / 80)
        y = int(j * img.size[1] / 80)
        pixel = img.getpixel((x, y))
        if pixel == [0, 0, 0, 255].all():
            row.append(1)
        else:
            row.append(0)
    grid.append(row)

for i in range(len(grid)):
    for j in range(len(grid[0])):
        print(grid[i][j], end = '')
    print()

img = Image.open('MHK.png')
width, height = img.size
print("Kích thước của tệp MHK.png là", width, "x", height)
