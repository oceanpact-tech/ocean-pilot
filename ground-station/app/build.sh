if [ ! -d "build/bin" ]; then
    mkdir -p build/bin
fi

cmake -Bbuild -S.
cmake --build build -j$(nproc)