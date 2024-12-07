Welcome to our TNP project

To compile please do the following

mkdir build
cd build
cmake ..
make

You have now compiled the project

To be able to use it please use the following command:

./ransac filename.obj numberofplanes normals

If the term normals is added at the end then the calculations will
try to use the normals that are in the object file for the ransac algorithm.

If you use the following:

./ransac filename.obj numberofplanes

This will do the ransac without using the normals of the object.

The word "numberofplanes" here represents the number that corresponds
to the amount of planes the user wants in return.

exemple d'utilisation de commande:

./ransac data/road_small.obj 1

./ransac data/church.obj 8 normals