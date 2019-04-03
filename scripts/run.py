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
interUserConnections = ["false"];
stepLengthEstimation = ["false"];
mapKeepPercent = [1.0];

# For chosen parameters
for (iUC, mKP, sLE) in zip(interUserConnections, mapKeepPercent, stepLengthEstimation):

    # Changing parameters to selected values
    setYamlFile(parameterFileName, "interUserConnections ", iUC);
    setYamlFile(parameterFileName, "mapKeepPercent ", mKP);
    setYamlFile(parameterFileName, "stepLengthEstimation ", sLE);

    # Path depending on the parameters
    dir = "iUC_" + str(iUC) + "_mKP_" + str(mKP) + "_sLE_" + str(sLE);

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
