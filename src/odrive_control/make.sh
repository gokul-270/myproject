rm -v  build/*.o
rm -v bin/Y*

echo "Compiling odrive_can_functions.cpp"
g++ -g -Wall -Iinclude -I/usr/include -c src/odrive_can_functions.cpp	-o build/odrive_can_functions.o $OPTIONS

echo "Compiling src/misc_functions.cpp	"
g++ -g -Wall -Iinclude -I/usr/include -c src/misc_functions.cpp	-o build/misc_functions.o     $OPTIONS

#echo "Compiling src/request_msg.cpp	"
#g++ -g -Wall -Iinclude -I/usr/include -c src/request_msg.cpp	-o build/request_msg.o     $OPTIONS

echo "Compiling src/debug_print.cpp	"
g++ -g -Wall -Iinclude -I/usr/include -c src/debug_print.cpp	-o build/debug_print.o     $OPTIONS

echo "Compiling src/task.cpp	"
g++ -g -Wall -Iinclude -I/usr/include -c src/task.cpp	-o build/task.o     $OPTIONS

echo "Compiling odrive_tests.cpp "
g++ -g -Wall -I./include -I./src -I/usr/include -c src/odrive_tests.cpp -o build/odrive_tests.o -L/usr/lib -lm -lpthread $OPTIONS

echo "Compiling main.cpp "
g++ -g -Wall -I./include -I./src -I/usr/include -c src/main.cpp -o build/main.o -L/usr/lib -lm -lpthread $OPTIONS

echo "building bin/YanthraCanBusInterface "
g++ -Wall -o bin/YanthraCanBusInterface \
            build/main.o \
            build/odrive_can_functions.o \
            build/task.o  \
            build/misc_functions.o \
            build/debug_print.o   -L/usr/lib -lm -lpthread $OPTIONS

