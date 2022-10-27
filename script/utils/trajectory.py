

def load_trajectory(path):
    f = open(path,mode='r')
    pose = {}
    cnt=0
    while True:
        cnt+=1
        line = f.readline().split(' ')
        if cnt == 1:
            continue
        if line[0] != 'VERTEX_SE3:QUAT':
            break
        pose[int(line[1])] = [float(l) for l in line[2:]]
        print('loading {}th pose'.format(cnt))
    return pose
