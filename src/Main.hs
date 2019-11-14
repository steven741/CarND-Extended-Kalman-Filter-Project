module Main where

import App
import EKF

app conn = do
  sensor <- receive conn
  send conn Response {rX = 0.0, rY = 0.0}
  print sensor
  app conn

main =
  runApp app
