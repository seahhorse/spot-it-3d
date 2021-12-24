import json
import cv2

session = "OC6_1D2C480pB"
cam_no = "1"

def get_coord(event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDOWN:
        if cam_no == "1":
            entry["Cam " + cam_no].append({"Bottom-Right": "(" + str(x+15 - 640) + ", " + str(y+15) + ")", "Top-Left": "(" + str(x-15 - 640) + ", " + str(y-15) + ")"})
        else:
            entry["Cam " + cam_no].append({"Bottom-Right": "(" + str(x+15) + ", " + str(y+15) + ")", "Top-Left": "(" + str(x-15) + ", " + str(y-15) + ")"})

g = []

f = open('../data/log/'+ session +'_targets-2d-out.json')
data = json.load(f)

cap = cv2.VideoCapture('../data/output/'+ session +'_ann.avi')


for frame_no in range(1, len(data) + 1):

    _, frame = cap.read()
    cv2.imshow('frame', frame)

    entry = {"Cam " + cam_no: [], "Frame Number": frame_no}
    cv2.setMouseCallback('frame', get_coord)

    while True:
        keypress = cv2.waitKey(0) & 0xFF
        if keypress == 107: # keypress 'k' to keep original
            g.append(data[frame_no-1])
            print(g[frame_no-1])
            break
        elif keypress == 97: # keypress 'a' to append to original
            g.append(data[frame_no-1])
            for track in entry["Cam " + cam_no]:
                g[frame_no-1]["Cam " + cam_no].append(track)
            print(g[frame_no-1])
            break
        elif keypress == 32: # keypress '<space>' to write over original
            g.append(entry)
            print(g[frame_no-1])
            break
        elif keypress == 113: # keypress 'q' to quit
            exit()

cap.release()
cv2.destroyAllWindows()


with open('../data/log/'+ session +'_true_'+cam_no+'.json', 'w') as outfile:
    json.dump(g, outfile, indent=4)


