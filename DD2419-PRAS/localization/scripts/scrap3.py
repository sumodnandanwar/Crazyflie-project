#!/usr/bin/env python
from os.path import expanduser
import csv

def Dict_goal():
    global dictgoal
    dictgoal = []
    pathcsv = expanduser('~')
    pathcsv += '/dd2419_ws/src/DD2419-PRAS/localization/scripts/Pathcsv'
    with open(pathcsv, 'rb') as csv_file:
        csv_reader = csv.reader(csv_file)
        next(csv_reader)
        for line in csv_reader:
            line[0] = float(line[0])
            line[1] = float(line[1])
            dictgoal.append(line)
    print dictgoal[0][1]
    return dictgoal

def main():
    Dict_goal()

if __name__ == '__main__':
    main()