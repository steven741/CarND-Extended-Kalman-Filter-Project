# CarND-Extended-Kalman-Filter-Project
This is a completed implementation of the extended kalman filter project for Udacity's Self-Driving 🚗 Nanodegree using Haskell. This project was made to be used with the [Udacity's Term 2 Self Driving Car Simulator](https://github.com/udacity/self-driving-car-sim/releases).

## Setup, Build and, Run

* Install [Haskell Tool Stack](https://docs.haskellstack.org/en/stable/README/)

```bash
stack build
stack exec ekf
```

or

```bash
stack run
```


## Basic Concept Of Kalman Filters

Kalman filters are a class of information filters that can filter noise from measurments. When given enough information kalman filters often produce useful estimations. The source of the information that kalman filters use usually come from mathematical models and sensor measurments. The sensor measurments that kalman filters update on need not be from one source. That is to say that a kalman filter may update it's belief based sensor measurments of various types. This concept, of using multiple sources of information, is sometimes called sensor fusion.

A kalman filter can be understood generally as having 2 components: a state space and an error space. The state space is an estimate of the state we wish to know. For example, if we'd like to know the position of an object the state space may contain some `x, y, z` position values. Or, if we'd like to know the fuel level of a gas tank the state space may contain some `l` value of the remaining liters of fuel in a tank. The error space captures the variance or noise of the information. This space provides the certainty of belief of the filter's estimation.


## Intuition For The Need Of Linear Models

## Outline Of Implementation

