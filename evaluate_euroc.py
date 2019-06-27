import os
import sys
import associate
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from align import *

#support format: timestamp tx ty tz qx qy qz qw

# file state_groundtruth_estimate0/data.csv
def format_euroc(file, save_file):
    fout = open(save_file, 'w')
    for line in open(file).readlines():
        if line[0] != '#' and len(line)>0:
            v = line.strip().split(',')
            fout.write('%f %s %s %s %s %s %s %s\n'%(float(v[0])/1e9,v[1],v[2],v[3],v[5],v[6],v[7],v[4]))

# file vio.txt
def format_own(file, save_file):
    fout = open(save_file, 'w')
    for line in open(file).readlines():
        if line[0] != '#' and len(line)>0:
            v = line.strip().split(' ')
            if len(v) > 8:
                fout.write('%s %s %s %s %s %s %s %s\n'%(v[0],v[1],v[2],v[3],v[5],v[6],v[7],v[4]))

def ate(first_file, second_file, offset = 0, max_diff = 0.02, scale = 1.0, plot = 1, save_png = None):
    first_list = associate.read_file_list(first_file)
    second_list = associate.read_file_list(second_file)

    matches = associate.associate(first_list, second_list,float(offset),float(max_diff))    
    if len(matches)<2:
        sys.exit("Couldn't find matching timestamp pairs between groundtruth and estimated trajectory! Did you choose the correct sequence?")

    first_xyz = np.matrix([[float(value) for value in first_list[a][0:3]] for a,b in matches]).transpose()
    second_xyz = np.matrix([[float(value)*float(scale) for value in second_list[b][0:3]] for a,b in matches]).transpose()
    rot,trans,trans_error = align(second_xyz, first_xyz)

    first_stamps = first_list.keys()
    first_stamps.sort()
    first_xyz_full = np.matrix([[float(value) for value in first_list[b][0:3]] for b in first_stamps]).transpose()

    second_stamps = second_list.keys()
    second_stamps.sort()
    second_xyz_full = np.matrix([[float(value)*float(scale) for value in second_list[b][0:3]] for b in second_stamps]).transpose()
    second_xyz_full_aligned = rot * second_xyz_full + trans

    rmse = np.sqrt(np.dot(trans_error,trans_error) / len(trans_error))
    emean = np.mean(trans_error)
    emedian = np.median(trans_error)
    estd = np.std(trans_error)
    emin = np.min(trans_error)
    emax = np.max(trans_error)

    name = os.path.basename(first_file)[:-4]

    if save_png is not None or plot:
        fig = plt.figure()
        ax = fig.add_subplot(111)
        plot_traj(ax,first_stamps,first_xyz_full.transpose().A,'-',"red","ground truth")
        plot_traj(ax,second_stamps,second_xyz_full_aligned.transpose().A,'-',"blue","estimated")
        ax.legend()
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        plt.title(name)
    if save_png is not None:
        plt.savefig(save_png)
    if plot:
        plt.show()
    return len(trans_error), rmse, emean, emedian, estd, emin, emax

def cost(res_file):
    ts = [float(x.strip().split(' ')[-1]) for x in open(res_file).readlines() if x.strip()[0]!='#']
    ts = ts[int(len(ts)*0.1):]
    return np.mean(ts), np.min(ts), np.max(ts)

def single_evaluate(res_file, gt_file, save_png = None):
    out_res_file = os.path.join(os.path.dirname(res_file),
                                os.path.basename(res_file).replace('out_','est_'))
    format_own(res_file, out_res_file)
    return ate(gt_file, out_res_file, plot = False, save_png = save_png) + cost(res_file)

def batch_evaluate(res_dir, gt_dir):
    eval_table = {}
    for file in sorted(os.listdir(res_dir)):
        if file[:4]=='out_' and file[-4:]=='.txt':
            name = file[4:-4]
            gt_file = os.path.join(gt_dir, '%s.txt'%name)
            if os.path.exists(gt_file):
                eval_table[name] = single_evaluate(os.path.join(res_dir, file), gt_file,
                                    os.path.join(res_dir,'traj_%s.png'%name))
    if len(eval_table.keys()) > 0:
        save_data = pd.DataFrame(eval_table)
        save_data.index = ['N','rmse','mean','median','std','min','max','tmean','tmin','tmax']
        save_data = save_data.T
        save_data.to_csv(os.path.join(vio_res_dir, 'result.csv'))
        print(save_data[['rmse','min','max','N','tmean']])
    else:
        print("No valid trajectory 'out_*.txt' in: %s"%res_dir)


if __name__ == '__main__':
    # Usage: python evaluate_euroc.py <vio_result_dir> <EuRoC zip dir>
    vio_res_dir = '/home/symao/Desktop/msckf'
    gt_dir = os.path.join(sys.path[0],'groundtruth')
    if len(sys.argv) > 1:
        vio_res_dir = sys.argv[1]

    batch_evaluate(vio_res_dir,gt_dir)
