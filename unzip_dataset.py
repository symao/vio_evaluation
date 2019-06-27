import os

if __name__ == '__main__':
    zips = [x for x in os.listdir('./') if x[-4:]=='.zip']
    for z in zips:
        os.system('unzip -o -d %s %s'%(z[:-4], z))