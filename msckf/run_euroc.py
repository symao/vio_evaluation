import os
import sys
import time

# How to do before run this script?
# 1. go to code dir, type 'mkdir build; cd build; cmake ..; make -j' and wait for compilation done.
# 2. Download EuRoC dataset of ASL format in '.zip' format rather than '.bag' format.
# 3. modify whole_dir, output_file, res_dir in this script.
# 4. run script and wait, result will be saved into res_dir.

def single_run_euroc(msckf_dir, data_dir, outfile):
    print('Run EuRoC data: %s'%data_dir)
    t1 = time.time()
    fexe = os.path.join(msckf_dir,'build/demo_euroc')
    fparam = os.path.join(msckf_dir, 'param/param_euroc.yaml')

    cmd_list = (fexe,fparam,data_dir)
    for f in cmd_list:
        if not os.path.exists(f):
            sys.exit('File not exists:%s'%f)
    os.system('%s %s %s'%cmd_list)
    time.sleep(10)
    os.system('mv vio.txt %s'%outfile)
    print('Done. %.2f Seconds.'%(time.time()-t1))

def batch_run_euroc(msckf_dir, whole_dir, res_dir, select_dataset = None):
    if not os.path.exists(res_dir):
        os.makedirs(res_dir)

    if select_dataset is None:
        select_dataset = [x for x in os.listdir(whole_dir) if os.path.exists(os.path.join(whole_dir,x,'mav0'))]

    for name in select_dataset:
        path = os.path.join(whole_dir, name)
        out_file = os.path.join(res_dir, 'out_%s.txt'%name)
        single_run_euroc(msckf_dir, path, out_file)

if __name__ == '__main__':
    msckf_dir = '/home/symao/workspace/msckf/'
    whole_dir = '/home/symao/data/euroc/zip'
    res_dir = './results'
    # data_list = ['MH_01_easy', 'MH_02_easy', 'MH_03_medium', 'MH_04_difficult', 'MH_05_difficult',
    #              'V1_01_easy', 'V1_02_medium', 'V1_03_difficult',
    #              'V2_01_easy', 'V2_02_medium', 'V2_03_difficult']

    batch_run_euroc(msckf_dir, whole_dir, res_dir)