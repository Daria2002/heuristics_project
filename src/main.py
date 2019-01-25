#!/usr/bin/env python
from LoadData import *
import sys
from HeuristicAlgorithmClass import *
from os.path import dirname, abspath


def write_to_file(file_name, best_positions, execution_time):
    path = dirname(dirname(abspath(__file__)))
    txt_file_name = "res-" + execution_time + "m" + "-" + file_name + ".txt"
    path_txt_file = os.path.join(path, txt_file_name)
    file = open(path_txt_file, "w+")
    for i, line in enumerate(best_positions):
        if i > 0:
            file.write("\n")
        if len(best_positions[line]) == 0:
            continue
        for vehicle in best_positions[line]:
            file.write(str(vehicle))
            file.write(" ")
    file.close()


def write_info_message_in_txt():
    path = dirname(dirname(abspath(__file__)))
    txt_file_name = "res-" + execution_time + "m" + "-" + file_name + ".txt"
    path_txt_file = os.path.join(path, txt_file_name)
    file = open(path_txt_file, "w+")
    file.write("No solution found")
    file.close()


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print('Load file with data and execution time')
    if len(sys.argv) > 3:
        print('Load only one file and execution time!')

    file_name = str(sys.argv[1])
    execution_time = str(sys.argv[2])  # time in minutes
    data = LoadDataClass(file_name)
    data.get_data()
    heuristicAlgorithm = HeuristicAlgorithmClass(data, execution_time)
    best_positions = dict()
    if heuristicAlgorithm.run():
        best_positions = heuristicAlgorithm.get_best_positions()
        write_to_file(file_name, best_positions, execution_time)
    else:
        write_info_message_in_txt()
        print('No solution found')
