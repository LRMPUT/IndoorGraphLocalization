# This file is part of the IndoorGraphLocalization distribution (https://github.com/LRMPUT/IndoorGraphLocalization).
# Copyright (c) 2018 Michal Nowicki (michal.nowicki@put.poznan.pl)
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, version 3.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

from subprocess import call
import sys
import os
import fileinput


summary = []
for root, dirs, files in os.walk("results/"):

    for name in files:
        if 'trajEvalOut.txt' in name:
            configName = root.split("/")[1];
            #print configName

            data = [];
            with open(root + "/" + name) as file:
                data = file.readlines()

            result = data[-1];
            result = result.replace('\t', ' ').replace(',', '').replace('\n', '').split(' ');

            RMSE = float(result[3]);
            AvgErr = float(result[6]);
            Sigma = float (result[8]);
            MaxErr = float(result[10]);

            summary.append([configName, RMSE, AvgErr, Sigma, MaxErr]);


summary = sorted(summary, key=lambda summary: summary[0]);

groups = [];

print "Config".ljust(40) + "RMSE".ljust(8) + "AvgErr".ljust(8) + "Sigma".ljust(8) + "MaxErr".ljust(8)
for line in summary:
    k = line[0].rfind('_');
    groupName = line[0][:k];
    if groupName not in groups:
        groups.append(groupName);

    print str(line[0]).ljust(40) + str(line[1]).ljust(8) + str(line[2]).ljust(8) + str(line[3]).ljust(8) + str(line[4]).ljust(8)

print "--------------"

for group in groups:
    RMSE = 0;
    AvgErr = 0;
    Sigma = 0;
    MaxErr = 0;
    count = 0;

    for line in summary:
        if group in line[0]:
            RMSE = RMSE + line[1];
            AvgErr = AvgErr + line[2];
            Sigma = Sigma + line[3];
            MaxErr = MaxErr + line[4];
            count = count + 1;

    print group.ljust(40) + str(round(RMSE/count,2)).ljust(8) + str(round(AvgErr/count,2)).ljust(8) + str(round(Sigma/count,2)).ljust(8) + str(round(MaxErr/count,2)).ljust(8)

