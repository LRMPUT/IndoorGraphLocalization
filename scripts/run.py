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
from itertools import product

def setYamlFile(filePathWithName, parameter, value):
    print
    "Setting " + parameter + " = " + str(value);

    outLines = []
    with open(filePathWithName, 'r') as f_in:
        for line in f_in:
            if parameter in line:
                outLines.append(parameter + str(value) + "\n");
            else:
                outLines.append("%s" % line);
    with open(filePathWithName, 'w') as f_out:
        for line in outLines:
            f_out.write(line);


# Path to save main results - create if needed
if not os.path.exists("results"):
    os.makedirs("results");
else:
    call('rm -r results/*', shell=True);

# Name of the parameter file
parameterFileName = "parameters.txt";

# -------------------------------------------
# Parameters:
#runsPerSequence = 50;
#interUserConnections = ["false", "true", "false", "true", "false", "true", "false", "true", "false", "true", "false", "true"];
#mapKeepPercent = [0.5, 0.5, 0.6, 0.6, 0.7, 0.7, 0.8, 0.8, 0.9, 0.9, 1.0, 1.0];
#stepLengthEstimation = ["true", "true", "true", "true", "true", "true", "true", "true", "true", "true", "true", "true"];

runsPerSequence = 1;

# interUserConnections = ["false", "false", "false", "false", "false", "false", "false", "false", "false", "false"];
# stepLengthEstimation = ["false", "false", "false", "false", "false", "false", "false", "false", "false", "false"];
# mapKeepPercent = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0];
# vprWeight = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0];
# wallWeight = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
# fa_safetyThresholdRatio = [1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1];
# fa_earlyAcceptedVicinity = [5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5];
# fa_consistencyThreshold = [3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3];
# fa_acceptedVicinityThreshold = [10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10];

interUserConnections = ["false"];
stepLengthEstimation = ["false"];
mapKeepPercent = [1.0];
vprWeight = [10.0];
wallWeight = [0.0];
wallVicinityThreshold = [0.1];
wallInitType = [0];
fa_compareLength = [50];
fa_safetyThresholdRatio = [1.1];
fa_earlyAcceptedVicinity = [5];
fa_consistencyThreshold = [3];
fa_acceptedVicinityThreshold = [10];


# For chosen parameters
# for (iUC, mKP, sLE, vpr, wall, faSTR, faEAC, faCT, faAVT) in zip(interUserConnections, mapKeepPercent, stepLengthEstimation,
#                                                                      vprWeight, wallWeight,fa_safetyThresholdRatio, fa_earlyAcceptedVicinity,
#                                                                      fa_consistencyThreshold, fa_acceptedVicinityThreshold):

for (iUC, mKP, sLE, vpr, wW, wVT, wIT, faCL, faSTR, faEAC, faCT, faAVT) in product(interUserConnections, mapKeepPercent,
                                                                           stepLengthEstimation,
                                                                           vprWeight, wallWeight, wallVicinityThreshold,
                                                                           wallInitType, fa_compareLength,
                                                                           fa_safetyThresholdRatio,
                                                                           fa_earlyAcceptedVicinity,
                                                                           fa_consistencyThreshold,
                                                                           fa_acceptedVicinityThreshold):


    # Changing parameters to selected values
    setYamlFile(parameterFileName, "interUserConnections ", iUC);
    setYamlFile(parameterFileName, "mapKeepPercent ", mKP);
    setYamlFile(parameterFileName, "stepLengthEstimation ", sLE);
    setYamlFile(parameterFileName, "EDGE_VPR_INF_MAT_WEIGHT ", vpr);
    setYamlFile(parameterFileName, "EDGE_WALL_PENALTY ", wW);
    setYamlFile(parameterFileName, "wallVicinityThreshold ", wVT);
    setYamlFile(parameterFileName, "wallInitType ", wIT);
    setYamlFile(parameterFileName, "FASTABLE_compareLength ", faCL);
    setYamlFile(parameterFileName, "FASTABLE_safetyThresholdRatio ", faSTR);
    setYamlFile(parameterFileName, "FASTABLE_earlyAcceptedVicinity ", faEAC);
    setYamlFile(parameterFileName, "FASTABLE_consistencyThreshold ", faCT);
    setYamlFile(parameterFileName, "FASTABLE_acceptedVicinityThreshold ", faAVT);


    # Path depending on the parameters
    dir = "iUC_" + str(iUC) + "_mKP_" + str(mKP) + "_sLE_" + str(sLE) + "_vpr_" + str(vpr) + "_wW_" + str(wW) + \
        "_wVT_" + str(wVT) + "_faCL_" + str(faCL) + "_faSTR_" + str(faSTR) + "_faEAC_" + str(faEAC) + "_faCT_" + \
          str(faCT) + "_faAVT_" + str(faAVT);

    # For all selected sequences
    for i in range(0, runsPerSequence):

        # full path
        fullDir = dir + "_run_" + str(i) + "/";

        # Create dir
        if not os.path.exists("results/" + fullDir):
            os.makedirs("results/" + fullDir);
        else:
            call('rm results/' + fullDir + '/*', shell=True);

        # Copy parameters
        call('cp ' + parameterFileName  + ' results/' + fullDir + '/', shell=True);

        # Run code
        call('./build/graph_localization', shell=True);

        # Create trajectories
        call('python scripts/visGraphResultSensors.py > trajEvalOut.txt', shell=True);

        # Copy results
        call('cp parameters.txt results/' + fullDir + '/', shell=True);
        call('mv output.g2o results/' + fullDir + '/', shell=True);
        call('mv trajEvalOut.txt results/' + fullDir + '/', shell=True);
        call('mv gtMap.png results/' + fullDir + '/', shell=True);
        call('mv traj*.png results/' + fullDir + '/', shell=True);
        call('mv wifi.png results/' + fullDir + '/', shell=True);
        call('mv errors.txt results/' + fullDir + '/', shell=True);

call('python scripts/summarizeResults.py', shell=True);
