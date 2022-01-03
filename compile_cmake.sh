set -e
trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT

mkdir -p build
cd build
cmake ../
make
mv spot-it-3d ../
cd ../classifier_service
python3 main.py &>/dev/null &
cd ../
# ./spot-it-3d