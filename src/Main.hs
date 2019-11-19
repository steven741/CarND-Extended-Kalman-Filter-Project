module Main where

import App
import EKF

import Numeric.LinearAlgebra ((!))


app conn = do
  sensor <- recv conn
  app' conn (makeFilter sensor)

app' conn kf = do
  sensor <- recv conn

  let kf' = update sensor kf
  let px = (kf_x kf') ! 0
  let py = (kf_x kf') ! 1

  send conn Response { rX = px
                     , rY = py }
  app' conn kf'

main =
  runApp app
