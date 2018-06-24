# CarND-Extended-Kalman-Filter-Project
This is a completed implementation of the extended kalman filter project for Udacity's Self-Driving 🚗 Nanodegree using Haskell. This project was made to be used with the [Udacity's Term 2 Self Driving Car Simulator](https://github.com/udacity/self-driving-car-sim/releases) and [websocketd](http://websocketd.com/).

# Setup
🐧 Fedora:
```bash
sudo dnf install haskell-platform
```
```bash
cabal install aeson linear
```

# Build and Run
From the root directory of the project run:
```bash
ghc -O2 -dynamic src/Main.hs -o bin/main && rm -rf src/Main.hi src/Main.o
```
Launch the Udacity simulator and run this command.
```bash
./bin/websocketd --port=4567 ./bin/main
```