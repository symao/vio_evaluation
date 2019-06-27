import os

def collect(name, src_path, tar_path):
    if not os.path.exists(tar_path):
        os.makedirs(tar_path)

    for p in [x for x in os.listdir(src_path) if x[-4:]=='.png' and x[:5]=='traj_']:
        src_name = os.path.join(src_path, p)
        tar_name = os.path.join(tar_path, p[5:-4]+"_%s.png"%name)
        os.rename(src_name, tar_name)

if __name__ == '__main__':
    collect('msckf', 'msckf/results', 'results')
    collect('vins', 'vins/results', 'results')
    collect('svo2', 'svo2/results', 'results')
    collect('smsckf', 'smsckf/results', 'results')
    collect('orbslam', 'orbslam/results', 'results')