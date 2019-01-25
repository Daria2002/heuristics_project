#!/usr/bin/env python
import os


class LoadDataClass:
    def __init__(self, file_name):
        self.projectPath = os.path.abspath(os.path.pardir)
        self.filePath = os.path.join(self.projectPath, file_name)
        self.instanceFile = open(self.filePath, 'r')
        self.dataInFile = self.instanceFile.readlines()
        self.numOfVehicles = 0
        self.numOfLines = 0
        self.vehiclesLength = []
        self.vehicleType = []
        self.constraints = dict()
        self.linesLength = []
        self.leavingTime = []
        self.scheduleType = []
        self.blockedLines = dict()

    def get_num_of_vehicles(self):
        num_of_vehicles_string = ''
        for element in self.dataInFile[0]:
            if element == '\n':
                break
            num_of_vehicles_string += element
        return int(num_of_vehicles_string)

    def get_num_of_lines(self):
        return int(self.dataInFile[1])

    def get_vehicle_length(self):
        help_length = ''
        vehicles_length = []
        for length in self.dataInFile[3]:
            if length == '\n':
                break
            if length != " ":
                help_length += length
            else:
                vehicles_length.append(int(help_length))
                help_length = ''
                if len(vehicles_length) >= self.numOfVehicles:
                    break
        return vehicles_length

    def get_vehicle_type(self):
        help_type = ''
        vehicle_type = []
        for type_of_vehicle in self.dataInFile[5]:
            if type_of_vehicle == '\n':
                break
            if type_of_vehicle != " ":
                help_type += type_of_vehicle
            else:
                vehicle_type.append(int(help_type))
                help_type = ''
                if len(vehicle_type) >= self.numOfVehicles:
                    break
        return vehicle_type

    def get_constraints(self):
        i = 7
        constraints = dict()
        while i < 7 + self.numOfVehicles:
            help_array = []
            for el in self.dataInFile[i]:
                if el == '\n':
                    break
                if el == ' ':
                    continue
                help_array.append(int(el))
            if len(help_array) > self.numOfLines:
                break
            constraints[i - 7] = help_array
            i += 1
        return constraints

    def get_lines_length(self):
        help_length = ''
        lines_length = []
        for length in self.dataInFile[9 - 1 + self.numOfVehicles]:
            if length == '\n':
                break
            if length != " ":
                help_length += length
            else:
                lines_length.append(int(help_length))
                help_length = ''
                if len(lines_length) >= self.numOfLines:
                    break
        return lines_length

    def get_leaving_time(self):
        help_leaving_time = ''
        leaving_time = []
        for leavingTime in self.dataInFile[11 - 1 + self.numOfVehicles]:
            if leavingTime == '\n':
                break
            if leavingTime != " ":
                help_leaving_time += leavingTime
            else:
                leaving_time.append(int(help_leaving_time))
                help_leaving_time = ''
                if len(leaving_time) >= self.numOfVehicles:
                    break
        return leaving_time

    def get_schedule_type(self):
        help_schedule_type = ''
        schedule_type = []
        for scheduleType in self.dataInFile[13 - 1 + self.numOfVehicles]:
            if scheduleType == '\n':
                schedule_type.append(int(help_schedule_type))
                break
            if scheduleType != " ":
                help_schedule_type += scheduleType
            else:
                schedule_type.append(int(help_schedule_type))
                help_schedule_type = ''
                if len(schedule_type) >= self.numOfVehicles:
                    break
        return schedule_type

    def get_blocked_lines(self):
        blocked_lines = dict()
        i = 15 - 1 + self.numOfVehicles
        blocker = 0
        while i < len(self.dataInFile):
            blocker_loaded = False
            blocked_list = []
            help_element = ''
            for el in self.dataInFile[i]:
                if el == '\n':
                    break
                if el != ' ':
                    help_element += el
                    continue
                elif blocker_loaded:
                    blocked_list.append(int(help_element))
                    help_element = ''
                elif not blocker_loaded:
                    blocker_loaded = True
                    blocker = help_element
                    help_element = ''

            if len(help_element) > 0:
                blocked_list.append(int(help_element))

            blocked_lines[int(blocker)] = blocked_list
            i += 1
        return blocked_lines

    def get_data(self):
        self.numOfVehicles = self.get_num_of_vehicles()
        self.numOfLines = self.get_num_of_lines()
        self.vehiclesLength = self.get_vehicle_length()
        self.vehicleType = self.get_vehicle_type()
        self.constraints = self.get_constraints()
        self.linesLength = self.get_lines_length()
        self.leavingTime = self.get_leaving_time()
        self.scheduleType = self.get_schedule_type()
        self.blockedLines = self.get_blocked_lines()