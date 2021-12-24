import json

def centroid(br, tl):
    return ((br[0] + tl[0]) // 2, (br[1] + tl[1]) // 2)

def within(cent, br, tl):
    return cent[0] >= tl[0] and cent[0] <= br[0] and cent[1] >= tl[1] and cent[1] <= br[1]

def inc_avg(orig, data, count):
    return orig + ((data-orig)/count)

def evaluate(session, cam_nos):

    f_pred = open('../data/log/'+ session +'_targets-2d-out.json')
    data_pred = json.load(f_pred)
    f_act = open('../data/log/'+ session +'_true.json')
    data_act = json.load(f_act)

    AP = []
    AR = []
    AF = []

    for cameras in range(len(cam_nos)):

        cam_no = "Cam " + str(cam_nos[cameras])

        for frame_no in range(1, len(data_pred) + 1):
            
            TP = 0
            FP = len(data_pred[frame_no-1][cam_no])
            FN = len(data_act[frame_no-1][cam_no])
            
            for detection in data_pred[frame_no-1][cam_no]:
                d_br = [int(s) for s in detection["Bottom-Right"][1:-1].split(", ") if s.isdigit()]
                d_tl = [int(s) for s in detection["Top-Left"][1:-1].split(", ") if s.isdigit()]
                
                for truth_n in range(FN):
                    truth = data_act[frame_no-1][cam_no][truth_n]
                    t_br = [int(s) for s in truth["Bottom-Right"][1:-1].split(", ") if s.isdigit()]
                    t_tl = [int(s) for s in truth["Top-Left"][1:-1].split(", ") if s.isdigit()]    

                    if within(centroid(d_br,d_tl), t_br, t_tl):
                        TP += 1
                        FP -= 1
                        FN -= 1

                        data_act[frame_no-1][cam_no].remove(truth)
                        break
            # print(f"Frame: {frame_no}, TP: {TP}, FP: {FP}, FN: {FN}")
            if (TP+FP != 0):
                AP.append(TP/(TP+FP))
            if (TP+FN != 0):
                AR.append(TP/(TP+FN))
            if (TP+FP+FN != 0):
                AF.append((2 * TP) / (2 * TP + FP + FN))

    print(f"Session = {session}, Precision = {(sum(AP)/len(AP)):.3f}, Recall = {(sum(AR)/len(AR)):.3f}, F1 = {(sum(AF)/len(AF)):.3f} ")


sessions = [("1_1D1C720pA", [0]), 
            ("2_1D1C720pB", [0]), 
            ("3_2D1C720pA", [0]), 
            ("4_2D1C720pB", [0]), 
            ("5_1D2C720pA", [0,1]), 
            ("6_1D2C720pB", [0,1]), 
            ("7_2D2C720pA", [0,1]), 
            ("8_2D2C720pB", [0,1]), 
            ("9_1D1C480pA", [0]), 
            ("10_1D1C480pB", [0]), 
            ("11_2D1C480pA", [0]), 
            ("12_2D1C480pB", [0]), 
            ("13_1D2C480pA", [0,1]), 
            ("14_1D2C480pB", [0,1]), 
            ("15_2D2C480pA", [0,1]), 
            ("16_2D2C480pB", [0,1]),
            ("OC1_1D1C720pA", [0]),
            ("OC2_1D1C720pB", [0]),
            ("OC3_1D1C480pA", [0]),
            ("OC4_1D1C480pB", [0]),
            ("OC5_1D2C480pA", [0,1]),
            ("OC6_1D2C480pB", [0,1]),]

for session, cam_nos in sessions:
    evaluate(session, cam_nos)