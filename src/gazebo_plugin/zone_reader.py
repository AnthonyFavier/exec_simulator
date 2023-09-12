import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from typing import Any, Dict, List, Tuple
from progress.bar import IncrementalBar


PATH = "/home/afavier/new_exec_sim_ws/src/gazebo_plugin/"
FILENAME = "zones.png"

#load the original image
img_rgb = mpimg.imread(PATH+FILENAME)[...,:3]
 
def is_white(p):
    return p[0]==1.0 and p[1]==1.0 and p[2]==1.0


class Segment:
    def __init__(self, y) -> None:
        self.y = y          
        self.x1 = -1   
        self.x2 = -1   

    def __repr__(self) -> str:
        return f"({self.y},{self.x1}-{self.x2})"

class Zone:
    def __init__(self) -> None:
        self.id = -1
        # Right-Upper corner (x1,y1)
        self.x1 = -1   
        self.y1 = -1   
        # Left-Lower corner (x2,y2)
        self.x2 = -1   
        self.y2 = -1   

    def __repr__(self) -> str:
        return f"({self.x1}-{self.y1},{self.x2}-{self.y2})"

# Extract Segments
segments = [] #type: List[Segment]
bar = IncrementalBar("Extracting Segments", max=len(img_rgb))
for y in range(len(img_rgb)):
    in_zone = False
    for x in range(len(img_rgb[0])):
        pix = img_rgb[y][x]

        # find start seg
        if in_zone==False and not is_white(pix):
            in_zone = True
            curr_seg = Segment(y)
            curr_seg.x1 = x

        # find end seg
        if in_zone==True and is_white(pix):
            in_zone = False
            curr_seg.x2 = x-1
            segments.append(curr_seg)

    bar.goto(y)
bar.goto(bar.max)
bar.finish()
# print("Segments:", segments)

zones = [] #type: List[Zone]
bar = IncrementalBar("Extracting Zones", max=len(segments))
while len(segments)!=0:
    seg = segments.pop(0)
    
    # start new zone
    zone = Zone()
    zone.x1 = seg.x1
    zone.x2 = seg.x2
    zone.y1 = seg.y

    # Seek end of zone (y)
    found = True
    c_y = seg.y
    while found:
        found = False
        c_y+=1
        for s in segments:
            if s.x1==zone.x1 and s.x2==zone.x2 and s.y==c_y:
                found = True
                segments.remove(s)
                break
    zone.y2 = c_y-1

    zones.append(zone)

    bar.next()
bar.goto(bar.max)
bar.finish()

# Update image
# Importing the PIL library
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

# Open an Image
img = Image.open(PATH+FILENAME)
 
# Call draw Method to add 2D graphics in an image
tmp_img = img.copy()
I1 = ImageDraw.Draw(tmp_img)
 
# Custom font style and font size
font_size = 65
myFont = ImageFont.truetype('FreeMono.ttf', font_size)
small_font_size = int(font_size/3)
mySmallFont = ImageFont.truetype('FreeMono.ttf', small_font_size)
print("Zones:")
for i,z in enumerate(zones):
    # s = f"{i}-({z.x1},{z.y1})-({z.x2},{z.y2})"
    s = f"{i},{z.x1},{z.y1},{z.x2},{z.y2}"
    print(f"\t{s}")
    I1.text( ( (z.x2+z.x1)/2, (z.y2+z.y1)/2 ), f"{i}", fill=(0,0,0), font=myFont, anchor='mm')
    I1.text( ( (z.x2+z.x1)/2, (z.y2+z.y1)/2+2*small_font_size ), f"({z.x1},{z.y1})-({z.x2},{z.y2})", fill=(0,0,0), font=mySmallFont, anchor='mm')

# Display edited image
tmp_img.show()

##################################################

# Demande renuméroté
new_zones = [] # List[Zone]
print("What shall be the final numbering?")
# for i,z in enumerate(zones):
#     print(f"{i} -> ", end="")
#     new_id = input()
#     z.id = int(new_id) if new_id!="" else i
print("new_id <- current_id")
for i in range(len(zones)):
    print(f"{i} <- ", end="")
    new_id = int(input())
    zones[i].id = new_id

def get_id_zone(z):
    return z.id
zones.sort(key=get_id_zone)

# Call draw Method to add 2D graphics in an image
I2 = ImageDraw.Draw(img)
 
# Add Text to an image
f = open(PATH+FILENAME[:-4]+"_coords.txt", 'w')

# Custom font style and font size
font_size = 65
myFont = ImageFont.truetype('FreeMono.ttf', font_size)
small_font_size = int(font_size/3)
mySmallFont = ImageFont.truetype('FreeMono.ttf', small_font_size)
print("Zones:")
for z in zones:
    # s = f"{i}-({z.x1},{z.y1})-({z.x2},{z.y2})"
    s = f"{z.id},{z.x1},{z.y1},{z.x2},{z.y2}"
    print(f"\t{s}")
    f.write(f"{s}\n")
    I2.text( ( (z.x2+z.x1)/2, (z.y2+z.y1)/2 ), f"{z.id}", fill=(0,0,0), font=myFont, anchor='mm')
    I2.text( ( (z.x2+z.x1)/2, (z.y2+z.y1)/2+2*small_font_size ), f"({z.x1},{z.y1})-({z.x2},{z.y2})", fill=(0,0,0), font=mySmallFont, anchor='mm')
f.close()

# Display edited image
img.show()

# Save the edited image
img.save(PATH+FILENAME[:-4]+"_numbered.png")

