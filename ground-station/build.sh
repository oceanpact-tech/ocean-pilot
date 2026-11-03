if [ ! -d "build/bin" ]; then
    mkdir -p build/bin
fi

cd build
cmake ..
make
cd ..