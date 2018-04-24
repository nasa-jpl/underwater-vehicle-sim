#!/bin/bash

rosdoc_lite -o ../underwater_vehicle_sim/doc ../underwater_vehicle_sim

rm index.html

echo "<!DOCTYPE html>" >> index.html
echo "<html>" >> index.html
echo "<body>" >> index.html

echo "<h2>Package Docs</h2>" >> index.html
echo "<p><a href=\"../underwater_vehicle_sim/doc/html/index.html\">underwater_vehicle_sim</a></p>" >> index.html

echo "</body>" >> index.html
echo "</html>" >> index.html
 
