
g++ -I./include -L./lib/hungarian -L./lib/json src/*.cpp lib/hungarian/Hungarian.cpp lib/json/jsoncpp.cpp -o spot-it-3d `pkg-config --cflags --libs opencv4`
for filename in data/input/* 
do
    echo "$filename"
    ./spot-it-3d "$filename"
    echo "$filename processing complete!"
done