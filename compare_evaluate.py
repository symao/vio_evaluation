import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import associate
from align import *

# compare rmse
def compare_rmse(namelist, dirlist, save_dir=None):
    if len(dirlist) < 2 or len(dirlist) != len(namelist):
        return
    rmse_table = []
    for name, path in zip(namelist, dirlist):
        if not os.path.isdir(path):
            print('Invalid path:%s'%path)
            continue
        fres = os.path.join(path,'result.csv')
        if not os.path.exists(fres):
            print('Read result failed. File not exists:%s'%fres)
            continue
        data = pd.read_csv(fres,index_col=0)
        rmse = data[['rmse']]
        rmse.columns = [name]
        rmse_table.append(rmse)

    rmse_table = pd.concat(rmse_table, axis=1)
    print('============ ATE (RMSE) =============')
    print(rmse_table)
    print('=====================================')

    if save_dir is not None:
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        rmse_table.to_csv(os.path.join(save_dir, 'ate.csv'))

def load_data(file):
    return np.array([[float(x) for x in s.strip().split(' ')] for s in open(file).readlines()])

# compare trajectory
def compare_traj(namelist, dirlist, gt_dir, save_dir=None, plot = False):
    if len(dirlist) < 2 or len(dirlist) != len(namelist):
        return

    if save_dir is not None:
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)

    valid_path = []
    for path in dirlist:
        files = [x for x in os.listdir(path) if x[:4]=='est_' and x[-4:]=='.txt']
        if not os.path.isdir(path) or len(files) == 0:
            print('Invalid path:%s'%path)
            continue
        valid_path.append(path)

    colors = ['red','blue','green','purple','pink','sienna','gray','yellow','black','gold','darkcyan']
    datanames = sorted([x[4:-4] for x in os.listdir(dirlist[0]) if x[:4]=='est_' and x[-4:]=='.txt'])
    for dataname in datanames:
        plt.figure()
        plt.title(dataname)
        gt_file = os.path.join(gt_dir,'%s.txt'%dataname)
        gt_list = associate.read_file_list(gt_file)
        gt_stamps = gt_list.keys()
        gt_stamps.sort()
        gt_traj = np.matrix([[float(value) for value in gt_list[b][0:3]] for b in gt_stamps]).transpose()
        plt.plot(gt_traj[0,:].T,gt_traj[1,:].T,colors[0],label='ground truth')
        plt.plot(gt_traj[0,0],gt_traj[1,0],'yp',markersize=8)

        cidx = 1
        for name, path in zip(namelist, dirlist):
            traj_file = os.path.join(path,'est_%s.txt'%dataname)
            if not os.path.exists(traj_file):
                continue
            traj_list = associate.read_file_list(traj_file)
            matches = associate.associate(gt_list, traj_list, 0, 0.02)
            if len(matches) < 2:
                continue
            gt_xyz = np.matrix([[float(value) for value in gt_list[a][0:3]] for a,b in matches]).transpose()
            traj_xyz = np.matrix([[float(value) for value in traj_list[b][0:3]] for a,b in matches]).transpose()
            rot,trans,trans_error = align(traj_xyz, gt_xyz)                                
            aligned_traj = rot * traj_xyz + trans
            if np.max(trans_error) < 100:
                plt.plot(aligned_traj[0,:].T,aligned_traj[1,:].T,colors[cidx%len(colors)],label=name)
            cidx+=1
        plt.legend()
        if save_dir is not None:
            plt.savefig(os.path.join(save_dir,'%s.png'%dataname))
    if plot:
        plt.show()

if __name__ == '__main__':
    save_dir = './output'
    gt_dir = os.path.join(sys.path[0], 'groundtruth')
    usage = 'Usage: python compare_evaluate.py alg1,alg2,alg3 path_to_alg1 path_to_alg2 path_to_alg3'
    
    if len(sys.argv) <= 3:
        sys.exit("Bad input: " + ' '.join(sys.argv[1:])+'\n'+usage)

    namelist = sys.argv[1].split(',')
    dirlist = sys.argv[2:]
    if len(namelist) != len(dirlist):
        sys.exit("Bad input: " + ' '.join(sys.argv[1:])+'\n'+usage)
    compare_rmse(namelist, dirlist, save_dir)
    compare_traj(namelist, dirlist, gt_dir, save_dir)
