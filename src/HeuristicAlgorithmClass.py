#!/usr/bin/env python
import itertools
import time

import random
import copy
import numpy as np


class HeuristicAlgorithmClass:
    def __init__(self, data, execution_time):
        self.execution_time = execution_time
        self.data = data
        self.free_space = copy.deepcopy(self.data.linesLength)
        self.usedSpaceInLine = []
        self.best_positions = dict()
        self.more_blockers_dict = dict()
        self.sorted_vehicles = []  # vehicles sorted in some line
        self.vehicle_with_constraints = []
        self.lines_with_constraints = []
        self.typeDict = dict()
        self.sortTypeTimeDict = dict()  # key=type, value=array of vehicles in line, time sorted
        self.lineType = []  # index=line-1, value=type
        self.filled_lines = []
        self.global_goal_evaluated = 0
        self.vehiclesInLines = {line: [] for line in range(1, self.data.numOfLines + 1)}
        self.best_first = 0
        self.best_second = 0
        self.schedule_types_in_lines = {line: [] for line in range(0, self.data.numOfLines)}

    def f1(self, vehicle_series_in_lines):
        different_series = 0
        used_lines = [i for i, line in enumerate(vehicle_series_in_lines) if line != 0]

        for i, line in enumerate(used_lines):
            if i + 1 >= len(used_lines):
                break
            if vehicle_series_in_lines[used_lines[i]] != vehicle_series_in_lines[used_lines[i + 1]]:
                different_series += 1
        return different_series

    def f2(self, vehicles_in_lines):
        """
        :param vehicles_in_lines:
        :return: number of used lines
        """
        used_lines = sum(1 if len(vehicles_in_lines[line]) > 0 else 0 for line in vehicles_in_lines)
        return used_lines

    def f3(self, positions):
        """
        :return:unused_space in used lines, extra_space without distance between 0.5
        """
        used_lines_length = 0  # only used lines
        total_vehicles_length = 0
        total_lines_length = 0
        for line in positions:
            for vehicle in positions[line]:
                total_vehicles_length += self.data.vehiclesLength[vehicle - 1]
            total_lines_length += self.data.linesLength[line - 1]
            if len(positions[line]) == 0:
                continue
            used_lines_length += self.data.linesLength[line - 1]
        used_lines = self.f2(positions)
        used_space_with_distance = total_vehicles_length + 0.5 * (self.data.numOfVehicles - used_lines)
        unused_space = used_lines_length - used_space_with_distance
        extra_space = total_lines_length - total_vehicles_length
        return unused_space, extra_space

    def first_global_goal(self, positions, vehicle_series_in_lines):
        """
        :return:
        """
        self.global_goal_evaluated += 1
        number_of_used_lines = self.f2(positions)
        unused_space, extra_space = self.f3(positions)
        p1 = 1. / (number_of_used_lines - 1)
        p2 = 1. / self.data.numOfLines
        p3 = 1. / extra_space
        return self.f1(vehicle_series_in_lines) * p1 + self.f2(positions) * p2 + unused_space * p3

    def g1(self, schedule_types_in_lines):
        same_schedule = 0
        for line in schedule_types_in_lines:
            i = 0
            length = len(schedule_types_in_lines[line])
            while i < length:
                if i + 1 >= length:
                    i += 1
                    break
                elif schedule_types_in_lines[line][i] == schedule_types_in_lines[line][i + 1]:
                    same_schedule += 1
                    i += 1
                    continue
                i += 1
        return same_schedule

    def g2(self, schedule_types_in_lines):
        """
        :return: number of neighbour lines where last line in previous line has same schedule
        type as line in next line
        """
        same_schedule = 0
        used_keys = []
        for line in schedule_types_in_lines:
            if len(schedule_types_in_lines[line]) > 0:
                used_keys.append(line)
        for i, usedKey in enumerate(used_keys):
            if i + 1 >= len(used_keys):
                break
            length = len(schedule_types_in_lines[used_keys[i]])
            if schedule_types_in_lines[used_keys[i]][length - 1] == \
                    schedule_types_in_lines[used_keys[i + 1]][0]:
                same_schedule += 1
        return same_schedule

    def g3(self, vehicles_in_lines):
        n_total = 0
        n = 0
        for line in vehicles_in_lines:
            if len(vehicles_in_lines[line]) < 2:  # if there is no car in line or only one
                continue
            vehicles = vehicles_in_lines[line]
            for i, vehicle in enumerate(vehicles):
                if i + 1 >= len(vehicles):
                    break
                first_vehicle = self.data.leavingTime[vehicles[i] - 1]
                second_vehicle = self.data.leavingTime[vehicles[i + 1] - 1]
                vr = second_vehicle - first_vehicle
                if 10 <= vr <= 20:
                    n = 15
                elif vr > 20:
                    n = 10
                elif vr < 10:
                    n = -4 * (10 - vr)
                n_total += n
        return n_total

    def second_global_goal(self, positions, schedule_types_in_lines):
        vehicles_in_lines = 0
        num_of_neighbours = 0
        number_of_used_lines = self.f2(positions)
        for line in positions:
            vehicles_in_lines += len(positions[line])
        for line in positions:
            neighbours_in_line = 0
            for vehicle in positions[line]:
                neighbours_in_line += 1
            if len(positions[line]) == 0:
                continue
            num_of_neighbours += neighbours_in_line - 1
        r1 = 1. / (vehicles_in_lines - number_of_used_lines)
        r2 = 1. / (number_of_used_lines - 1)
        r3 = 1. / (15 * num_of_neighbours)
        return self.g1(schedule_types_in_lines) * r1 + self.g2(schedule_types_in_lines) * r2 + self.g3(positions) * r3

    def make_type_dict(self):
        """
        set type_dict, key is type and value are vehicles of that type
        :return:
        """
        type_dict = {vehicle: [] for vehicle in self.data.vehicleType}
        [type_dict[vehicle_type].append(i) for i, vehicle_type in enumerate(self.data.vehicleType)]
        return type_dict

    def sort_type_dict(self):
        """
        set sort_type_time_dict, key is type and value are vehicles of that type, where vehicles are sorted depending
        on leaving time
        :return:
        """
        sorted_type = []
        sort_type_time_dict = {vehicle: [] for vehicle in self.data.vehicleType}
        for type in self.data.vehicleType:
            help_dict = dict()  # key is vehicle, value is leavingTime
            if type in sorted_type:
                continue
            sorted_type.append(vehicle for vehicle in self.typeDict[type])
            help_dict = {vehicle: self.data.leavingTime[vehicle] for vehicle in self.typeDict[type]}
            help_dict = sorted(help_dict, key=lambda vehicle: help_dict[vehicle])
            help_array = [vehicle for vehicle in help_dict]
            sort_type_time_dict[type] = help_array
        return sort_type_time_dict

    def add_random_type_to_lines(self):
        line_type = []
        type_array = [line_type for line_type in self.typeDict]
        unused_types = copy.deepcopy(type_array)
        while unused_types:
            unused_types = copy.deepcopy(type_array)
            line_type = []
            for line in range(0, self.data.numOfLines):
                random_type = random.choice(type_array)
                if random_type in unused_types:
                    unused_types.remove(random_type)
                line_type.append(random_type)
        return line_type

    def make_lines_with_constraints(self):
        lines_with_constraints = []
        for vehicle in self.data.constraints:
            for i, constraint in enumerate(self.data.constraints[vehicle]):
                if constraint == 0:
                    already_in_set = False
                    self.vehicle_with_constraints.append(vehicle)
                    for lineWithConstraints in lines_with_constraints:
                        if lineWithConstraints == i:
                            already_in_set = True
                            break
                    if not already_in_set:
                        lines_with_constraints.append(i)
        return lines_with_constraints

    def refresh_free_space(self, line, vehicle):
        self.free_space[line] = self.free_space[line] - self.data.vehiclesLength[vehicle] - 0.5

    def refresh_free_space_replace(self, line, vehicle_to_leave, vehicle_to_come, free_space):
        free_space[line - 1] = free_space[line - 1] + self.data.vehiclesLength[vehicle_to_leave - 1] - \
                               self.data.vehiclesLength[vehicle_to_come - 1]
        return free_space

    def change_vehicles_possible(self, line, vehicle_to_leave, vehicle_to_come, free_space):
        if (free_space[line - 1] + self.data.vehiclesLength[vehicle_to_leave - 1]
                - self.data.vehiclesLength[vehicle_to_come - 1] < 0):
            return False
        return True

    def make_more_blockers_dict(self):
        """
        make dictionary where key is blocked line and value are blockers
        self.more_blockers_dict contains only blocked lines blocked by more than one line
        :return:
        """
        blocked_lines = []
        more_blockers = []
        for blocker in self.data.blockedLines:
            for blocked in self.data.blockedLines[blocker]:
                if blocked in blocked_lines:
                    more_blockers.append(blocked)
                    continue
                blocked_lines.append(blocked)
        if len(more_blockers) == 0:
            return 0

        more_blockers_dict = {blocked - 1: [] for blocked in more_blockers}
        for blocker in self.data.blockedLines:
            for blocked in self.data.blockedLines[blocker]:
                if blocked in more_blockers:
                    more_blockers_dict[blocked - 1].append(blocker - 1)
        return more_blockers_dict

    def fill_lines_with_constraints(self):
        for line_with_constraint in self.lines_with_constraints:  # self.lines_with_constraints = [0, self.numOfLines-1]
            if line_with_constraint in self.more_blockers_dict:
                continue

            for blocker in self.data.blockedLines:
                if line_with_constraint == blocker - 1:
                    break
                skip = False
                for blocked in self.data.blockedLines[blocker]:
                    if line_with_constraint == blocked - 1:
                        skip = True
                        break
                if skip:
                    break

            for vehicle in self.typeDict[self.lineType[line_with_constraint]]:
                if self.data.constraints[vehicle][line_with_constraint] == 0 or \
                        self.data.vehiclesLength[vehicle] > self.free_space[line_with_constraint] \
                        or vehicle in self.sorted_vehicles:
                    continue

                random_choice = random.uniform(-1, 2)
                if random_choice < 0:
                    continue

                self.vehiclesInLines[line_with_constraint + 1].append(vehicle + 1)
                self.refresh_free_space(line_with_constraint, vehicle)
                self.sorted_vehicles.append(copy.deepcopy(vehicle))
            if len(self.vehiclesInLines[line_with_constraint + 1]) > 1:
                self.vehiclesInLines = self.set_leaving_time_order_in_line(line_with_constraint, self.vehiclesInLines)

    def fill_blocked_and_blocker_lines(self):
        # fill lines blocked by more than one line
        if self.more_blockers_dict:
            for blocked in self.more_blockers_dict:  # self.lines_with_constraints = [0, self.numOfLines-1]
                for vehicle in reversed(self.sortTypeTimeDict[self.lineType[blocked]]):
                    if self.data.constraints[vehicle][blocked] == 0 or \
                            self.data.vehiclesLength[vehicle] > self.free_space[blocked] \
                            or vehicle in self.sorted_vehicles:
                        continue
                    self.vehiclesInLines[blocked + 1].append(vehicle + 1)
                    self.refresh_free_space(blocked, vehicle)
                    self.sorted_vehicles.append(vehicle)
                if len(self.vehiclesInLines[blocked + 1]) > 1:
                    self.vehiclesInLines = self.set_leaving_time_order_in_line(blocked, self.vehiclesInLines)

                for blocker in self.more_blockers_dict[blocked]:
                    for vehicle in self.sortTypeTimeDict[self.lineType[blocker]]:
                        if self.data.constraints[vehicle][blocker] == 0 or \
                                self.data.vehiclesLength[vehicle] > self.free_space[blocker] \
                                or vehicle in self.sorted_vehicles:
                            continue
                        if len(self.vehiclesInLines[blocked + 1]) > 0:
                            if self.data.leavingTime[vehicle] > \
                                    self.data.leavingTime[self.vehiclesInLines[blocked + 1][0] - 1]:
                                continue

                        random_choice = random.uniform(-1, 2)
                        if random_choice < 0:
                            continue

                        self.vehiclesInLines[blocker + 1].append(vehicle + 1)
                        self.refresh_free_space(blocker, vehicle)
                        self.sorted_vehicles.append(vehicle)
                    if len(self.vehiclesInLines[blocker + 1]) > 1:
                        self.vehiclesInLines = self.set_leaving_time_order_in_line(blocker, self.vehiclesInLines)

        # fill lines blocked by only one line
        for blocker_line in self.data.blockedLines:  # self.lines_with_constraints = [0, self.numOfLines-1]
            blocker = blocker_line - 1
            if self.more_blockers_dict:
                if blocker in self.more_blockers_dict:
                    continue
            skip = False
            if self.more_blockers_dict:
                for blocked_ in self.more_blockers_dict:
                    skip = False
                    for blocker_ in self.more_blockers_dict[blocked_]:
                        if blocker == blocker_:
                            skip = True
                            break
                    if skip:
                        break
            if skip:
                continue
            for vehicle in self.sortTypeTimeDict[self.lineType[blocker]]:
                if self.data.constraints[vehicle][blocker] == 0 or \
                        self.data.vehiclesLength[vehicle] > self.free_space[blocker] or \
                        vehicle in self.sorted_vehicles:
                    continue
                self.vehiclesInLines[blocker + 1].append(vehicle + 1)
                self.refresh_free_space(blocker, vehicle)
                self.sorted_vehicles.append(vehicle)

            for blockedLine in self.data.blockedLines[blocker + 1]:
                blocked = blockedLine - 1
                if self.more_blockers_dict:
                    if blocked in self.more_blockers_dict:
                        continue
                for vehicle in reversed(self.sortTypeTimeDict[self.lineType[blocked]]):
                    if self.data.constraints[vehicle][blocked] == 0 or \
                            self.data.vehiclesLength[vehicle] > self.free_space[blocked] \
                            or vehicle in self.sorted_vehicles:
                        continue
                    if len(self.vehiclesInLines[blocker + 1]) > 0:
                        blocker_length = len(self.vehiclesInLines[blocker + 1])
                        if self.data.leavingTime[vehicle] < \
                                self.data.leavingTime[self.vehiclesInLines[blocker + 1][blocker_length - 1] - 1]:
                            continue

                    random_choice = random.uniform(-1, 2)
                    if random_choice < 0:
                        continue

                    self.vehiclesInLines[blocked + 1].append(vehicle + 1)
                    self.refresh_free_space(blocked, vehicle)
                    self.sorted_vehicles.append(vehicle)
            for line in range(0, self.data.numOfLines):
                if len(self.vehiclesInLines[line + 1]) > 1:
                    self.vehiclesInLines = self.set_leaving_time_order_in_line(line, self.vehiclesInLines)

    def fill_simple_lines(self):
        for line in range(0, self.data.numOfLines):  # self.lines_with_constraints = [0, self.numOfLines-1]
            if self.more_blockers_dict:
                if line in self.more_blockers_dict:
                    continue
            skip = False
            for blocker in self.data.blockedLines:
                skip = False
                if line == blocker - 1:
                    skip = True
                    break
                for blocked in self.data.blockedLines[blocker]:
                    if line == blocked - 1:
                        skip = True
                        break
                if skip:
                    break

            if skip:
                continue

            random_vehicle = random.uniform(-5, 5)
            vehicle_found = False
            if random_vehicle > 0:
                for vehicle in self.sortTypeTimeDict[self.lineType[line]]:
                    if self.data.constraints[vehicle][line] == 0 or \
                            self.data.vehiclesLength[vehicle] > self.free_space[line] \
                            or vehicle in self.sorted_vehicles:
                        continue

                    vehicle_found = False
                    self.vehiclesInLines[line + 1].append(vehicle + 1)
                    self.refresh_free_space(line, vehicle)
                    self.sorted_vehicles.append(vehicle)

            if not vehicle_found:
                for vehicle in reversed(self.sortTypeTimeDict[self.lineType[line]]):
                    if self.data.constraints[vehicle][line] == 0 or \
                            self.data.vehiclesLength[vehicle] > self.free_space[line] \
                            or vehicle in self.sorted_vehicles:
                        continue

                    self.vehiclesInLines[line + 1].append(vehicle + 1)
                    self.refresh_free_space(line, vehicle)
                    self.sorted_vehicles.append(vehicle)

            for line in range(0, self.data.numOfLines):
                if len(self.vehiclesInLines[line + 1]) > 1:
                    self.vehiclesInLines = self.set_leaving_time_order_in_line(line, self.vehiclesInLines)

    def check_blocked_and_blocker_lines(self, vehicles_in_lines):
        for blocker in self.data.blockedLines:
            for blocked in self.data.blockedLines[blocker]:
                for vehicles_in_blocked in vehicles_in_lines[blocked]:
                    for vehicles_in_blocker in vehicles_in_lines[blocker]:
                        if self.data.leavingTime[vehicles_in_blocker - 1] > \
                                self.data.leavingTime[vehicles_in_blocked - 1]:
                            return False
        return True

    def search_better(self, best_rate, vehicles_in_lines, schedule, free_space):
        vehicles = np.linspace(1, self.data.numOfVehicles, self.data.numOfVehicles)
        all_combinations = itertools.combinations(vehicles, 2)
        best_positions = dict()
        first_index = 0
        first_line = 0
        second_line = 0
        second_index = 0

        for i, combination in enumerate(list(all_combinations)):
            if self.data.vehicleType[int(combination[0] - 1)] != self.data.vehicleType[int(combination[0] - 1)] \
                    or int(combination[0]) == int(combination[1]):
                continue
            first_vehicle = 0
            second_vehicle = 0
            for line in vehicles_in_lines:
                for index, vehicle in enumerate(vehicles_in_lines[line]):
                    if vehicle == int(combination[0]):
                        first_line = line
                        first_index = index
                        first_vehicle = vehicle
                    elif vehicle == int(combination[1]):
                        second_line = line
                        second_index = index
                        second_vehicle = vehicle

            change_possible1 = self.change_vehicles_possible(first_line, first_vehicle, second_vehicle, self.free_space)
            change_possible2 = self.change_vehicles_possible(second_line, second_vehicle, first_vehicle,
                                                             self.free_space)

            if first_vehicle == 0 or second_vehicle == 0:
                continue

            if self.data.constraints[first_vehicle - 1][second_line - 1] == 0 or \
                    self.data.constraints[second_vehicle - 1][first_line - 1] == 0:
                continue

            if self.data.vehicleType[first_vehicle - 1] != self.data.vehicleType[second_vehicle - 1]:
                change_possible1 = False

            if not change_possible1 or not change_possible2:
                continue

            free_space = self.refresh_free_space_replace(first_line, first_vehicle, second_vehicle, free_space)
            free_space = self.refresh_free_space_replace(second_line, second_vehicle, first_vehicle, free_space)

            vehicles_in_lines[first_line][first_index] = int(combination[1])
            vehicles_in_lines[second_line][second_index] = int(combination[0])

            schedule[first_line - 1][first_index] = self.data.scheduleType[int(combination[1] - 1)]
            schedule[second_line - 1][second_index] = self.data.scheduleType[int(combination[0] - 1)]

            vehicle_series_in_lines = [
                0 if len(vehicles_in_lines[line]) == 0 else self.data.vehicleType[vehicles_in_lines[line][0] - 1] for
                line in vehicles_in_lines]

            finish_first_global_goal = self.first_global_goal(vehicles_in_lines, vehicle_series_in_lines)
            finish_second_global_goal = self.second_global_goal(vehicles_in_lines, schedule)

            rate = finish_second_global_goal * 1. / finish_first_global_goal

            if rate > best_rate:
                best_rate = rate
                best_positions = copy.deepcopy(vehicles_in_lines)
                continue

            vehicles_in_lines[first_line][first_index] = int(combination[0])
            vehicles_in_lines[second_line][second_index] = int(combination[1])
            schedule[first_line - 1][first_index] = self.data.scheduleType[int(combination[0] - 1)]
            schedule[second_line - 1][second_index] = self.data.scheduleType[int(combination[1] - 1)]
            free_space = self.refresh_free_space_replace(first_line, second_vehicle, first_vehicle, free_space)
            free_space = self.refresh_free_space_replace(second_line, first_vehicle, second_vehicle, free_space)

        return best_positions, free_space, schedule, best_rate

    def solution(self):
        self.typeDict = self.make_type_dict()
        self.sortTypeTimeDict = self.sort_type_dict()
        self.more_blockers_dict = self.make_more_blockers_dict()
        self.lines_with_constraints = self.make_lines_with_constraints()

        start = time.time()
        end = time.time()
        best_rate = -100
        self.best_positions = dict()
        while (end - start) - int(self.execution_time) * 60 < 0.1:
            block_lines_ok = False
            solution_ok = False
            while not block_lines_ok or not solution_ok:
                self.vehiclesInLines = dict()
                self.free_space = copy.deepcopy(self.data.linesLength)
                self.sorted_vehicles = []
                self.lineType = self.add_random_type_to_lines()
                for line in range(1, self.data.numOfLines + 1):
                    self.vehiclesInLines[line] = []

                random_choice = random.randint(0, 5)

                if random_choice % 2 == 0:
                    self.fill_lines_with_constraints()
                    self.fill_blocked_and_blocker_lines()
                    self.fill_simple_lines()

                elif random_choice % 2 == 1:
                    self.fill_blocked_and_blocker_lines()
                    self.fill_lines_with_constraints()
                    self.fill_simple_lines()

                end = time.time()  # seconds
                block_lines_ok = self.check_blocked_and_blocker_lines(self.vehiclesInLines)
                solution_ok = self.check_solution(self.vehiclesInLines)

                if (end - start) - int(self.execution_time) * 60 > 0.1:
                    break

            if block_lines_ok and solution_ok:
                self.vehicle_series_in_lines = []
                for line in self.vehiclesInLines:
                    if len(self.vehiclesInLines[line]) == 0:
                        self.vehicle_series_in_lines.append(0)
                        continue
                    self.vehicle_series_in_lines.append(self.data.vehicleType[self.vehiclesInLines[line][0] - 1])

                self.schedule_types_in_lines = self.set_schedule_types_in_lines(self.vehiclesInLines)
                first = self.first_global_goal(self.vehiclesInLines, self.vehicle_series_in_lines)
                second = self.second_global_goal(self.vehiclesInLines, self.schedule_types_in_lines)
                rate = second * 1. / first
                if rate > best_rate:
                    self.best_positions = copy.deepcopy(self.vehiclesInLines)
                    best_rate = rate
                    self.best_first = first
                    self.best_second = second
            self.schedule_types_in_lines = self.set_schedule_types_in_lines(self.vehiclesInLines)
            new_best_positions, new_free_space, schedule, check_best_rate = self.search_better(best_rate,
                                                                                               self.vehiclesInLines,
                                                                                               self.schedule_types_in_lines,
                                                                                               self.free_space)
            if new_best_positions:
                block_lines_ok = self.check_blocked_and_blocker_lines(new_best_positions)
                solution_ok = self.check_solution(new_best_positions)
                if block_lines_ok and solution_ok:
                    best_rate = check_best_rate
                    self.best_positions = copy.deepcopy(new_best_positions)
                    self.free_space = new_free_space
                    self.schedule_types_in_lines = schedule
                    self.schedule_types_in_lines = self.set_schedule_types_in_lines(self.best_positions)
                    self.vehicle_series_in_lines = []
                    for line in self.best_positions:
                        if len(self.best_positions[line]) == 0:
                            self.vehicle_series_in_lines.append(0)
                            continue
                        self.vehicle_series_in_lines.append(self.data.vehicleType[self.best_positions[line][0] - 1])
                    self.best_first = self.first_global_goal(self.best_positions, self.vehicle_series_in_lines)
                    self.best_second = self.second_global_goal(self.best_positions, self.schedule_types_in_lines)

        if not self.best_positions:
            return False
        return True

    def set_leaving_time_order_in_line(self, line, vehicles_in_lines):
        vehicle_time_dict = dict()
        for vehicle in vehicles_in_lines[line + 1]:
            vehicle_time_dict[vehicle] = self.data.leavingTime[vehicle - 1]
        vehicle_time_dict = sorted(vehicle_time_dict, key=lambda el: vehicle_time_dict[el])
        vehicles_in_lines[line + 1] = []
        for i, vehicle in enumerate(vehicle_time_dict):
            vehicles_in_lines[line + 1].append(vehicle)
        return vehicles_in_lines

    def calculate_used_space_in_lines(self):
        self.usedSpaceInLine = []
        for line in self.vehiclesInLines:
            vehicle_length_sum = 0
            for vehicle in self.vehiclesInLines[line]:
                vehicle_length_sum += self.data.vehiclesLength[vehicle - 1] + 0.5
            self.usedSpaceInLine.append(vehicle_length_sum - 0.5)
        return self.usedSpaceInLine

    def calculate_number_of_used_lines(self):
        number_of_lines = sum(1 if len(self.vehiclesInLines[line]) > 0 else 0 for line in self.vehiclesInLines)
        return number_of_lines

    def set_schedule_types_in_lines(self, vehicles_in_lines):
        schedule_types_in_lines = dict()
        for line in range(0, self.data.numOfLines):
            schedule_types_in_lines[line] = []
        for line in vehicles_in_lines:
            types = []
            for vehicle in self.vehiclesInLines[line]:
                types.append(self.data.scheduleType[vehicle - 1])
            schedule_types_in_lines[line - 1] = types
        return schedule_types_in_lines

    def get_best_positions(self):
        return self.best_positions

    def check_solution(self, vehicles_in_lines):
        sorted_vehicles = [vehicle for line in vehicles_in_lines for vehicle in vehicles_in_lines[line]]
        for vehicle in range(1, self.data.numOfVehicles + 1):
            vehicle_sorted = False
            for sorted_vehicle in sorted_vehicles:
                if vehicle == sorted_vehicle:
                    vehicle_sorted = True
                    break
            if not vehicle_sorted:
                return False
        return True

    def run(self):
        if self.solution():
            print('Global goal function evaluated: ', self.global_goal_evaluated, ' times')
            print('Best first global goal: ', self.best_first)
            print('Best second global goal: ', self.best_second)
            print('Best rate: ', self.best_second * 1. / self.best_first)
            return True
        return False
