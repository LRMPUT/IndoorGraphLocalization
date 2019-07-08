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

            summary.append([RMSE, AvgErr, Sigma, MaxErr, configName]);


summary = sorted(summary, key=lambda summary: summary[4]);

groups = [];

print "RMSE".ljust(8) + "AvgErr".ljust(8) + "Sigma".ljust(8) + "MaxErr".ljust(8),

parameters = ["iUC", "mKP", "sLE", "vpr", "wW", "wVT", "wIT", "wET", "faCL", "faSTR", "faEAC", "faCT", "faAVT"];

for param in parameters:
    print param.ljust(5),
print


for line in summary:
    k = line[4].rfind('_');
    groupName = line[4][:k];
    groupName = groupName[20:];
    if groupName not in groups:
        groups.append(groupName);

    print str(line[0]).ljust(8) + str(line[1]).ljust(8) + str(line[2]).ljust(8) + str(line[3]).ljust(8),

    paramValues = line[4].split('_');


    for p in parameters:
        index = paramValues.index(p) if p in paramValues else -1;
        if index == -1:
            print "-".ljust(5),;
        else:
            print str(paramValues[index + 1]).ljust(5),;
    print

print "--------------"

for group in groups:
    RMSE = 0;
    AvgErr = 0;
    Sigma = 0;
    MaxErr = 0;
    count = 0;

    for line in summary:
        if group in line[4]:
            RMSE = RMSE + line[0];
            AvgErr = AvgErr + line[1];
            Sigma = Sigma + line[2];
            MaxErr = MaxErr + line[3];
            count = count + 1;

    print str(round(RMSE/count,2)).ljust(8) + str(round(AvgErr/count,2)).ljust(8) + str(round(Sigma/count,2)).ljust(8) + \
          str(round(MaxErr/count,2)).ljust(8),\

    paramValues = group.split('_');

    for p in parameters:
        index = paramValues.index(p) if p in paramValues else -1;
        if index == -1:
            print "-".ljust(5),;
        else:
            print str(paramValues[index + 1]).ljust(5),;

    print

print "++++++++++"



vals = [40, 50];
for i in vals:

    RMSE = 0;
    AvgErr = 0;
    Sigma = 0;
    MaxErr = 0;
    count = 0;

    p = "faCL";
    paramValues = -1;

    for line in summary:

        paramValues = line[4].split('_');

        index = paramValues.index(p) if p in paramValues else -1;
        if index == -1:
            print "-".ljust(5),;
        else:
            paramV = int(paramValues[index + 1]);

            if paramV == i:
                RMSE = RMSE + line[0];
                AvgErr = AvgErr + line[1];
                Sigma = Sigma + line[2];
                MaxErr = MaxErr + line[3];
                count = count + 1;

    print p, i, round(RMSE/count,2), round(AvgErr/count,2)