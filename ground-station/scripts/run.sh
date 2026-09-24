cd bin

if [ $# -eq 0 ]; then
    echo "No arguments provided. Running the main program."
    ./ocean_pilot
else
    if [ $1 == "telemetry" ]; then
        echo "entrei aqui"
        ./ocean_pilot_test_telemetry "$@"
    fi
fi

cd ..