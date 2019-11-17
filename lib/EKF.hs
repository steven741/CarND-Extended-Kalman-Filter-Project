module EKF
  ( KF (..)

  , makeFilter
  ) where

import App (Sensor (..))

import Numeric.LinearAlgebra


data KF = KF
  { kf_x :: Vector Double
  , kf_p :: Matrix Double
  , kf_t :: Double
  } deriving Show


makeFilter (Laser px py ct) =
  KF { kf_x = vector [px,
                      py,
                      0,
                      0]
     , kf_p = matrix 4 [1, 0,    0,    0,
                        0, 1,    0,    0,
                        0, 0, 1000,    0,
                        0, 0,    0, 1000]
     , kf_t = ct }

makeFilter (Radar rho phi rho' ct) =
  KF { kf_x = vector [rho  * cos phi,
                      rho  * sin phi,
                      rho' * cos phi,
                      rho' * sin phi]
     , kf_p = matrix 4 [1, 0,    0,    0,
                        0, 1,    0,    0,
                        0, 0, 1000,    0,
                        0, 0,    0, 1000]
     , kf_t = ct }


{-
predict dt x p =
  let
    noiseAx = 9
    noiseAy = 9

    dt2     = dt ** 2
    dt3     = dt ** 3
    dt4     = dt ** 4

    f = (V4 (V4 1 0 dt  0)
            (V4 0 1  0 dt)
            (V4 0 0  1  0)
            (V4 0 0  0  1))
    q = (V4 (V4 ((dt4/4)*noiseAx)               0 ((dt3/2)*noiseAx) 0)
            (V4               0 ((dt4/4)*noiseAy)               0 ((dt3/2)*noiseAy))
            (V4 ((dt3/2)*noiseAx)               0   (dt2*noiseAx) 0)
            (V4               0 ((dt3/2)*noiseAy)               0 (dt2*noiseAy)))

    x' = f !* x
    p' = f !*! p !*! (transpose f) !+! q
  in
    (x', p')


mainLoop t x p = do
  msg <- getLine

  case getMsg msg of
    Nothing  -> skip
    Just obj -> filter $ getVals (measurement obj)
  where
    filter (Laser vals) =
      let
        px   = vals !! 0
        py   = vals !! 1
        curT = vals !! 2
        dt   = ((curT - t)/1000000.0)
      in
        if t == 0
        then
          mainLoop curT (V4 px py 0 0) p
        else
          let
            (x', p') = predict dt x p

            h = (V2 (V4 1 0 0 0)
                    (V4 0 1 0 0))
            r = (V2 (V2 0.0225 0.0000)
                    (V2 0.0000 0.0225))
            y = (V2 px py) - (h !* x')

            i  = identity
            ht = (transpose h)
            s  = h !*! p' !*! ht !+! r
            si = inv22 s
            k  = p' !*! ht !*! si

            x'' = x' + (k !* y)
            p'' = (i - k !*! h) !*! p'
          in do
            writeSim (x'' ^. _x) (x'' ^. _y)
            mainLoop curT x'' p''

    filter (Radar vals) =
      let
        rho  = vals !! 0
        phi  = vals !! 1
        rho' = vals !! 2
        curT = vals !! 3
        px   = rho * (cos phi)
        py   = rho * (sin phi)
        vx   = rho' * (cos phi)
        vy   = rho' * (sin phi)
        dt   = ((curT - t)/1000000.0)
      in
        if t == 0
        then
          mainLoop curT (V4 px py vx vy) p
        else
          let
            (x', p') = predict dt x p

            -- Calculate the Jacobian matrix values 
            px = x' ^. _x
            py = x' ^. _y
            vx = x' ^. _z
            vy = x' ^. _w

            c1 = (px ** 2) + (py ** 2)
            c2 = (sqrt c1)
            c3 = (c1 * c2);

            zRho  = c2;
            zPhi  = (atan2 py px)
            zRho' = (px*vx + py*vy) / zRho

            h = (V3 (V4 (px / c2) (py / c2) 0 0)
                    (V4 (-py / c1) (px / c1) 0 0)
                    (V4 (py*(vx * py - vy * px)/c3) (px*(vy * px - vx * py)/c3) (px/c2) (py/c2)))
            r = (V3 (V3 0.09 0.0000 0.00)
                    (V3 0.00 0.0009 0.00)
                    (V3 0.00 0.0000 0.09))
            z = (V3 rho phi rho') - (V3 zRho zPhi zRho')
            y = (V3  (z ^. _x) (atan2 (sin (z ^. _y)) (cos (z ^. _y))) (z ^. _z))

            i  = identity
            ht = (transpose h)
            s  = h !*! p' !*! ht !+! r
            si = inv33 s
            k  = p' !*! ht !*! si

            x'' = x' + (k !* y)
            p'' = (i - k !*! h) !*! p'
          in do
            writeSim (x'' ^. _x) (x'' ^. _y)
            mainLoop curT x'' p''
-}
