DEPENDENCIES=(
    libssl-dev
)

sudo apt update
sudo apt install -y "${DEPENDENCIES[@]}"

if [ -d "boost" ]; then
    if source boost/bootstrap.sh; then
        source boost/b2
    fi
fi

# The following directory should be added to compiler include paths: [pre-path to boost]/boost
# The following directory should be added to linker library paths:   [pre-path to boost]/boost/stage/lib
