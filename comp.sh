#comp.sh: successor to the original "make.bat -r" compilation helper script. 
#AN<13 May '22>: It seems that on Linux I need to include the "-lX11" link for SFML. How strange. 
#...             https://stackoverflow.com/questions/23585435/cannot-call-xinitthreads
#Written 12 May '22. 

g++ -std=c++20 -c "main.cpp" "/home/ktw/Corporate/Programming/c++/ktw-lib/ktwutil.cpp" "/home/ktw/Corporate/Programming/c++/ktw-lib/ktwgen.cpp" "/home/ktw/Corporate/Programming/c++/ktw-lib/ktwmath.cpp" -g
g++ -std=c++20 "main.o" "ktwutil.o" "ktwgen.o" "ktwmath.o" -o "main.out" -lsfml-graphics -lsfml-window -lsfml-system -lX11
rm -f "main.o" "ktwutil.o" "ktwgen.o" "ktwmath.o"

./main.out