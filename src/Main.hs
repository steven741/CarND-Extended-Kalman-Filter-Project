module Main where

import App
import EKF


app conn = do
  sensor <- recv conn
  app' conn (makeFilter sensor)

app' conn kf = do
  sensor <- recv conn
  send conn Response {rX = 0.0, rY = 0.0}
  print sensor
  app' conn kf

main =
  runApp app
