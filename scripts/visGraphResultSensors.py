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
import random
import math
import numpy as np
import sys
from subprocess import call
from PIL import Image, ImageTk, ImageDraw, ImageFont
import PIL
import matplotlib.pyplot as plt

def readPositions(dirName, fileName):
    scale = 0;
    with open("dataset/scale.map" , 'r') as f:
        for line in f:
            scale = float(line.strip('\n'))


    positions = [];
    pdrEdges = [];
    wknnEdges = [];
    vprEdges = [];

    with open(dirName + "/" + fileName , 'r') as f: 
        for line in f:
            lineSplitted = line.strip('\n').split(' ')
            if lineSplitted[0] == "VERTEX_SE2":
                id = int(lineSplitted[1]);
                X = float(lineSplitted[2]);
                Y = float(lineSplitted[3]);
                angle = float(lineSplitted[4]);
                positions.append([id, X, Y, angle]);

            elif lineSplitted[0] == "EDGE_PDR":
                id1 = int(lineSplitted[1]);
                id2 = int(lineSplitted[2]);
                pdrEdges.append([id1, id2]);
            elif lineSplitted[0] == "EDGE_WKNN" or lineSplitted[0] == "EDGE_WKNN_DEADZONE":
                #print lineSplitted
                id1 = int(lineSplitted[1]);
                id2 = int(lineSplitted[2]);
                id3 = int(lineSplitted[3]);
                id4 = int(lineSplitted[4]);
                id5 = int(lineSplitted[5]);
                wknnEdges.append([id1,id2,id3,id4,id5]);
            elif lineSplitted[0] == "EDGE_VPR":
                id1 = int(lineSplitted[1]);
                X = float(lineSplitted[2]);
                Y = float(lineSplitted[3]);
                vprEdges.append([id1,X,Y]);
       
    return scale, positions, pdrEdges, wknnEdges, vprEdges



def dot(a, b):
    return a[0] * b[0] + a[1] * b[1];

 
def dToSegment(segA, segB, p):
    ba = segB - segA;
    pa = segA - p;
 
    c = dot( ba, pa );
 
    # Closest point is a
    if c > 0.0 :
        return math.sqrt(dot( pa, pa ));
 
    bp = p - segB;
 
    # Closest point is b
    if dot( ba, bp ) > 0.0:
        return math.sqrt(dot( bp, bp ));
 
 
    # Closest point is between a and b
    e = pa - ba * (c / dot( ba, ba ));
    return math.sqrt(dot( e, e ));
 
def findXYTheta(positions, id):
    for pos in positions:
        if pos[0] == id:
            return pos[1], pos[2], pos[3]
    return 0, 0;


import colorsys


def HSVToRGB(h, s, v):
    (r, g, b) = colorsys.hsv_to_rgb(h, s, v)
    return (int(255 * r), int(255 * g), int(255 * b))


def getColor(x, n):
    huePartition = x / (n + 1.0)
    return HSVToRGB(huePartition, 1.0, 1.0)


def drawGT(dirName, metaGtPositions, metaGtLists, scale, outputFileName):
    gtCount = 14;

    image = Image.open("dataset/map.png").convert('RGB')
    basewidth = 3049
    # wpercent = (basewidth/float(image.size[0]))
    # hsize = int((float(image.size[1])*float(wpercent)))
    # image = image.resize((basewidth,hsize), PIL.Image.ANTIALIAS)
    wpercent = 1;
    draw = ImageDraw.Draw(image)

    lastPos = -1;
    color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255));
    colorNo = 1;

    # All of the GTS
    circleSize = 15;
    totalLen = 0;
    for trajId in range(0,gtCount):
        len = 0;
        for (id, list) in enumerate(gtPositions[trajId]):
            for id2 in gtList[trajId][id]:
                if id2>id:
                    prevX = gtPositions[trajId][id][0]* wpercent * scale;
                    prevY = gtPositions[trajId][id][1]* wpercent * scale;
                    nextX = gtPositions[trajId][id2][0]* wpercent * scale;
                    nextY = gtPositions[trajId][id2][1]* wpercent * scale;
                    draw.line((prevX, prevY, nextX, nextY), fill=color, width=15)

                    # if trajId == 0:
                    #     print gtPositions[trajId][id][0], gtPositions[trajId][id][1], gtPositions[trajId][id2][0], gtPositions[trajId][id2][1]

                    len = len + math.sqrt((gtPositions[trajId][id2][0] - gtPositions[trajId][id][0])* \
                                          (gtPositions[trajId][id2][0] - gtPositions[trajId][id][0]) + \
                                          (gtPositions[trajId][id2][1] - gtPositions[trajId][id][1])* \
                                          (gtPositions[trajId][id2][1] - gtPositions[trajId][id][1]));

        color = getColor(colorNo, gtCount);
        colorNo = colorNo + 1;
        draw.ellipse((35 - circleSize, 35 * colorNo - circleSize, 35 + circleSize, 35 * colorNo + circleSize),
                         fill=color,
                         outline='black')

        print("TrajID: %d\tLength: %.2f" % (trajId + 1, len))
        totalLen = totalLen + len;
    print("TrajALL: Length: %.2f" % (totalLen))
    image.save(outputFileName, "png")

def drawNodes(trajId, dirName, positions, pdrEdges, wknnEdges, vprEdges, metaGtPositions, metaGtLists, scale, outputFileName, outputFileName2,
              wallNode, wallNeighbours):
        
    image = Image.open("dataset/map.png").convert('RGB')
    basewidth = 3049
    #wpercent = (basewidth/float(image.size[0]))
    #hsize = int((float(image.size[1])*float(wpercent)))
    #image = image.resize((basewidth,hsize), PIL.Image.ANTIALIAS)
    wpercent = 1;
    draw = ImageDraw.Draw(image)

    # All of the GTS
    if trajId != -1:
        for (id, list) in enumerate(gtPositions[trajId]):
            for id2 in gtList[trajId][id]:
                if id2>id:
                    prevX = gtPositions[trajId][id][0]* wpercent * scale;
                    prevY = gtPositions[trajId][id][1]* wpercent * scale;
                    nextX = gtPositions[trajId][id2][0]* wpercent * scale;
                    nextY = gtPositions[trajId][id2][1]* wpercent * scale;
                    draw.line((prevX, prevY, nextX, nextY), fill='yellow', width=15)

    for (id, list) in enumerate(wallNeighbours):
        aX = wallNode[id][0] * wpercent * scale;
        aY = wallNode[id][1] * wpercent * scale;

        for item in list:
            bX = wallNode[item][0] * wpercent * scale;
            bY = wallNode[item][1] * wpercent * scale;

            draw.line((aX, aY, bX, bY), fill='cyan', width=15)


    count = 0;
    lastPosId = -1;
    for edge in pdrEdges:
        idPos = edge[0];

        if idPos - lastPosId != 0:
            count = count + 1;
        lastPosId = edge[1];
    #print count

    lastPos = -1;
    color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255));
    colorNo = 1;

    # TODO: Lets see if it is better with smaller circleSize - original 15
    circleSize = 10;
    for edge in pdrEdges:

        idPos = edge[0];
        curTrajId = int(idPos / 1000);

        if trajId == curTrajId or trajId == -1:
            xPos, yPos, thetaPos = findXYTheta(positions, idPos);
            xPos = xPos * wpercent * scale;
            yPos = yPos * wpercent * scale;
            xArrow = xPos + 2*circleSize*math.cos(thetaPos);
            yArrow = yPos + 2*circleSize*math.sin(thetaPos);

            idNextPos = edge[1];
            xNextPos, yNextPos, thetaPos = findXYTheta(positions, idNextPos);
            xNextPos = xNextPos * wpercent * scale;
            yNextPos = yNextPos * wpercent * scale;

            if idPos - lastPos != 0:
                # color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255));
                color = getColor(colorNo, count);
                colorNo = colorNo + 1;
                # color = (255, 0, 0);
                if trajId != -1:
                    color = (65, 105, 225);
                else:
                    draw.ellipse((35 - circleSize, 35*colorNo - circleSize, 35 + circleSize, 35*colorNo + circleSize), fill=color,
                             outline='black')

            draw.line((xPos, yPos, xNextPos, yNextPos), fill=color, width=5)
            draw.ellipse((xPos - circleSize, yPos - circleSize, xPos + circleSize, yPos + circleSize), fill=color,
                             outline='black')
            draw.line((xPos, yPos, xArrow, yArrow), fill=color, width=10)
            draw.ellipse((xNextPos - circleSize, yNextPos - circleSize, xNextPos + circleSize, yNextPos + circleSize), fill=color,
                             outline='black')
            lastPos = idNextPos;

    interUser = 0;
    total = 0;
    for edge in wknnEdges:
        idPos = edge[0];
        curTrajId = int(idPos / 1000);

        if trajId == curTrajId or trajId == -1:
            xPos, yPos, thetaPos = findXYTheta(positions, idPos);
            xPos = xPos * wpercent * scale;
            yPos = yPos * wpercent * scale;
            one = 0;
            for i in range (1,5):
                idWiFi = edge[i];
                xWiFi, yWiFi, thetaPos = findXYTheta(positions, idWiFi);
                xWiFi = xWiFi * wpercent * scale;
                yWiFi = yWiFi * wpercent * scale;

                if idWiFi < 100000:
                    one=1;
                    draw.line((xPos, yPos, xWiFi, yWiFi), fill='black', width=5)
                #     Not drawing WiFi WKNN to known map to make it more clear
                else:
                    draw.line((xPos, yPos, xWiFi, yWiFi), fill='red', width=3)
                
                    circleSize = 10;
                    draw.ellipse((xWiFi - circleSize, yWiFi - circleSize, xWiFi + circleSize, yWiFi + circleSize), fill='black',
                                 outline='black')
            if one==1:
                interUser = interUser+1
            else:
                total = total + 1;

    print "TrajID: " + str(trajId) + "\tinterUser: " + str(interUser) + " out of " + str(total) 

    for edge in vprEdges:
        idPos = edge[0];
        curTrajId = int(idPos / 1000);

        vprX = edge[1];
        vprY = edge[2];
        
        vprX = vprX * wpercent * scale;
        vprY = vprY * wpercent * scale;

        if trajId == curTrajId or trajId == -1:
            xPos, yPos, thetaPos = findXYTheta(positions, idPos);
            xPos = xPos * wpercent * scale;
            yPos = yPos * wpercent * scale;
         
            

            draw.line((xPos, yPos, vprX, vprY), fill='green', width=5)
            
            circleSize = 10;
            draw.ellipse((vprX - circleSize, vprY - circleSize, vprX + circleSize, vprY + circleSize), fill='green',
                                 outline='black')
        


    image.save(outputFileName, "png")


    ### DRAWING WIFI MAP
    if trajId == -1:
        image = Image.open("dataset/map.png").convert('RGB')
        basewidth = 3049
        wpercent = 1;
        draw = ImageDraw.Draw(image)

        for pos in positions:
            if pos[0] >= 100000:
                xWiFi = pos[1] * wpercent * scale;
                yWiFi = pos[2] * wpercent * scale;
                draw.ellipse((xWiFi - circleSize, yWiFi - circleSize, xWiFi + circleSize, yWiFi + circleSize), fill='red',
                             outline='black')

        for edge in wknnEdges:
            idPos = edge[0];
            curTrajId = int(idPos / 1000);

            xPos, yPos, thetaPos = findXYTheta(positions, idPos);
            xPos = xPos * wpercent * scale;
            yPos = yPos * wpercent * scale;
            one = 0;
            for i in range(1, 5):
                idWiFi = edge[i];
                xWiFi, yWiFi, thetaPos = findXYTheta(positions, idWiFi);
                xWiFi = xWiFi * wpercent * scale;
                yWiFi = yWiFi * wpercent * scale;

                if idWiFi < 100000:
                    one = 1;

            if one == 1:
                draw.ellipse(
                    (xPos - circleSize, yPos - circleSize, xPos + circleSize, yPos + circleSize),
                    fill=(65, 105, 225),
                    outline='black')


        image.save(outputFileName2, "png")


def readGroundTruth(dirName):

    gtPositions = [];
    gtList = [];
    for trajId in range (1,15):
        nodePositions = [];
        neighbourList = [];
        distance = 0;
        with open(dirName + "GT/" + str(trajId) + "/nodes.map", 'r') as f:
            nodeNo = int(f.readline());

            # lastX = -1;
            # lastY = -1;
            # start = 1;
            # print "NodeNo: ", nodeNo

            for i in range(0, nodeNo):
                lineSplitted = f.readline().strip('\n').split(' ')

                X = float(lineSplitted[0]);
                Y = float(lineSplitted[1]);
                nodePositions.append((X, Y));

                #print start
                # if start == 0:
                #     if trajId == 1:
                #         print lastX, lastY, X, Y
                #
                #     distance = distance + math.sqrt((lastX - X)*(lastX - X) + (lastY - Y)*(lastY - Y));
                #
                # else:
                #     start = 0;
                #
                # lastX = X;
                # lastY = Y;

                neighbourNo = int(f.readline());

                lineSplitted = f.readline().strip('\n').split(' ')
                neighboursForCurrentNode = []
                for neigh in lineSplitted:
                    #        print "Neighbour: " + str(neigh);
                    neighboursForCurrentNode.append(int(neigh));

                neighbourList.append(list(set(neighboursForCurrentNode)));


        gtPositions.append(nodePositions);
        gtList.append(neighbourList);
        # print trajId, " ", distance



    return gtPositions, gtList


def readWalls(dirName):
    nodePositions = [];
    neighbourList = [];
    with open(dirName + "/walls.map", 'r') as f:
        nodeNo = int(f.readline());
        # print "NodeNo: ", nodeNo

        for i in range(0, nodeNo):
            lineSplitted = f.readline().strip('\n').split(' ')

            X = float(lineSplitted[0]);
            Y = float(lineSplitted[1]);
            nodePositions.append((X, Y));

            neighbourNo = int(f.readline());

            lineSplitted = f.readline().strip('\n').split(' ')
            neighboursForCurrentNode = []
            for neigh in lineSplitted:
                #        print "Neighbour: " + str(neigh);
                neighboursForCurrentNode.append(int(neigh));

            neighbourList.append(list(set(neighboursForCurrentNode)));

    return nodePositions, neighbourList


def computeErrors(id, error):
    sum = 0;
    sum2 = 0;
    ile = 0;
    max = 0;
    for e in error:
        if e < 100:
            sum = sum + e;
            sum2 = sum2 + e * e;
            ile = ile + 1;
            if e > max:
                max = e;
    od = 0;
    if ile < 1:
        return;

    for e in error:
        if e < 100:
            od = od + (e - sum / ile) * (e - sum / ile)
    od = math.sqrt(od / (ile - 1))
    print("TrajID: %d\tRMSE: %.2f, Average error: %.2f, Sigma: %.2f, Max: %.2f, Third Quantile: %.2f" % (
    id+1, math.sqrt(sum2 / ile), sum / ile, od, max, np.quantile(error, 0.75)))

    with open("errors_" + str(id) + ".txt", "w") as text_file:
        for e in error:
            text_file.write(str(e) + '\n');


def errors(positions, gtPositions, gtList):
    errors = [[] for x in xrange(len(gtPositions))];



    for p in positions:

        trajId = int(p[0] / 1000);

        # It is not WIFI known location
        if p[0] < 100000:


            minDist = 999999999;
            for (id, list) in enumerate(gtPositions[trajId]):
                for idN in gtList[trajId][id]:
                    if idN > id:



                        dist = dToSegment(np.asarray(gtPositions[trajId][id]), np.asarray(gtPositions[trajId][idN]), np.asarray(p[1:3]));

                        # if trajId == 2:
                        #     print dist, gtPositions[trajId][id], gtPositions[trajId][idN], p[1:3]

                        if dist < minDist:
                            minDist = dist;
            #print "Distance: " + str(minDist)
            #print p
            errors[trajId].append(minDist)

    err = [];
    for i in range(0, len(errors)):
        computeErrors(i, errors[i])
        err = err + errors[i];
    print "-----------"
    computeErrors(-1, err)

########### MAIN ########
if len(sys.argv) == 1:


    firstMap = ".";
    scale, positions, pdrEdges, wknnEdges, vprEdges = readPositions('.', 'output.g2o')

    # Reading and drawing ground truth
    gtPositions, gtList = readGroundTruth('dataset/')
    wallPositions, wallNeighbours = readWalls('dataset/')
    drawGT(firstMap, gtPositions, gtList, scale, "gtMap.png")



    for trajId in range(0,len(gtPositions)):
        drawNodes(trajId, firstMap, positions, pdrEdges, wknnEdges, vprEdges, gtPositions, gtList, scale,
                  firstMap + "/traj_" + str(trajId+1) + ".png", "", wallPositions, wallNeighbours);

    drawNodes(-1, firstMap, positions, pdrEdges, wknnEdges, vprEdges, gtPositions, gtList, scale,
              firstMap + "/traj_all.png", firstMap + "/wifi.png", wallPositions, wallNeighbours);

    print "-----------"
    errors(positions, gtPositions, gtList)
