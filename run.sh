echo "Compiling Code...";

g++ -O3 -march=native -std=c++11 -mavx2 -mfma -fopenmp -fPIC -fno-math-errno -I/usr/local/include/eigen-3.4.0/ main.cpp -o main;

echo "Compile Done, running code...";

./main

echo "Creating Plots...";

python3 plot.py