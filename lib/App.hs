module App
  ( Message (..)
  , Response (..)
  , Sensor (..)

  , send
  , receive
  , runApp ) where

import Data.Aeson
import Data.Vector
import Data.ByteString.Lazy

import qualified  Network.WebSockets as WS


data Message = Message
  { mMeasurement :: String
  } deriving Show

data Response = Response
  { rX :: Double
  , rY :: Double
  } deriving Show

instance FromJSON Message where
  parseJSON (Array a)
    | a ! 0 == "telemetry" &&
      a ! 1 /= "empty"     &&
      a ! 1 /= Null = parseJSON (a ! 1)
    | otherwise     = mempty

  parseJSON (Object v) = do
    mMeasurement <- v .: "sensor_measurement"
    return $ Message mMeasurement

  parseJSON _ = mempty

instance ToJSON Response where
  toJSON r =
    Array $ fromList [ "estimate_marker"
                     , object [ "estimate_x" .= rX r
                              , "estimate_y" .= rY r
                              , "rmse_x"     .= (0.0 :: Double)
                              , "rmse_y"     .= (0.0 :: Double)
                              , "rmse_vx"    .= (0.0 :: Double)
                              , "rmse_vy"    .= (0.0 :: Double)]]


data Sensor
  = Laser
    { lPx :: Double
    , lPy :: Double
    , lCt :: Double }
  | Radar
    { rRho  :: Double
    , rPhi  :: Double
    , rRho' :: Double
    , rCt   :: Double }
  deriving Show

readSensor :: String -> Sensor
readSensor s
  | (Prelude.head s == 'L') =
    let
      px = vals !! 0
      py = vals !! 1
      ct = vals !! 2
    in
      Laser { lPx = px
            , lPy = py
            , lCt = ct }
  | (Prelude.head s == 'R') =
    let
      rho  = vals !! 0
      phi  = vals !! 1
      rho' = vals !! 2
      ct   = vals !! 3
    in
      Radar { rRho  = rho
            , rPhi  = phi
            , rRho' = rho'
            , rCt   = ct }
  where
    vals = Prelude.map read (words $ Prelude.tail s) :: [Double]


send :: WS.Connection -> Response -> IO ()
send conn msg =
  WS.sendTextData conn $ append "42" (encode msg)


receive :: WS.Connection -> IO Sensor
receive conn = do
  msgData <- WS.receiveData conn

  let msgCode = Data.ByteString.Lazy.take 2 msgData
  let msgBody = Data.ByteString.Lazy.drop 2 msgData

  if msgCode == "42" then
    case decode msgBody :: Maybe Message of
      Nothing -> continue
      Just msg -> return $ readSensor (mMeasurement msg)
  else
    continue
  where
    continue = do
       WS.sendTextData conn ("42[\"manual\",{}]" :: ByteString)
       receive conn


server :: (WS.Connection -> IO ()) -> WS.ServerApp
server app pending =
  WS.acceptRequest pending >>= app


runApp :: (WS.Connection -> IO ()) -> IO ()
runApp app =
  WS.runServer "127.0.0.1" 4567 (server app)
