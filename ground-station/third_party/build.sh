DEPENDENCIES=(
    libssl-dev
)

sudo apt update
sudo apt install -y "${DEPENDENCIES[@]}"

if [ -d "boost" ]; then
    if ! source boost/bootstrap.sh; then
        echo "Failed to bootstrap boost"
        exit -1
    fi
    
    if ! source boost/b2; then
        echo "Failed to build boost"
        exit -1
    fi
fi

# The following directory should be added to compiler include paths: [pre-path to boost]/boost
# The following directory should be added to linker library paths:   [pre-path to boost]/boost/stage/lib
