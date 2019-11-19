module EKF
  ( KF
  , kf_x
  , kf_p
  , kf_t

  , makeFilter
  , predict
  , update
  ) where

import Prelude hiding ((<>))

import App (Sensor (..))

import Numeric.LinearAlgebra


data KF = KF
  { kf_x :: Vector Double
  , kf_p :: Matrix Double
  , kf_t :: Word
  } deriving Show


makeFilter (Laser px py t) =
  KF { kf_x = 4 |> [px,
                    py,
                    0,
                    0]
     , kf_p = (4><4) [1, 0,    0,    0,
                      0, 1,    0,    0,
                      0, 0, 1000,    0,
                      0, 0,    0, 1000]
     , kf_t = t }

makeFilter (Radar rho phi rho' t) =
  KF { kf_x = 4 |> [rho  * cos phi,
                    rho  * sin phi,
                    rho' * cos phi,
                    rho' * sin phi]
     , kf_p =  (4><4) [1, 0,    0,    0,
                       0, 1,    0,    0,
                       0, 0, 1000,    0,
                       0, 0,    0, 1000]
     , kf_t = t }


predict :: Sensor -> KF -> (Vector Double, Matrix Double)
predict s kf =
  (x, p)
  where
    -- Basicly current time.
    sensorTime =
      case s of
        (Laser px py t) -> t
        (Radar rho phi rho' t) -> t

    -- Time inside the filter.
    filterTime =
      kf_t kf

    -- Delta-time. Essentially a timestep.
    dt =
      (fromIntegral (sensorTime - filterTime)) / 1000000.0

    noiseAx = 9
    noiseAy = 9

    dt2 = dt ** 2
    dt3 = dt ** 3
    dt4 = dt ** 4

    f = (4><4) [1, 0, dt,  0,
                0, 1,  0, dt,
                0, 0,  1,  0,
                0, 0,  0,  1]

    q = (4><4) [(dt4/4)*noiseAx, 0, (dt3/2)*noiseAx, 0,
                0, (dt4/4)*noiseAy, 0, (dt3/2)*noiseAy,
                (dt3/2)*noiseAx, 0, dt2*noiseAx, 0,
                0, (dt3/2)*noiseAy, 0, dt2*noiseAy]
    x = f #> kf_x kf
    p = f <> kf_p kf <> (tr f) + q


update (Laser px py t) kf =
  KF { kf_x = x'
     , kf_p = p'
     , kf_t = t }
  where
    (x, p) = predict (Laser px py t) kf

    h = (2><4) [1, 0, 0, 0,
                0, 1, 0, 0]
    r = (2><2) [0.0225, 0.0000,
                0.0000, 0.0225]
    y = 2 |> [px, py] - (h #> x)

    ht = tr h
    s  = h <> p <> ht + r
    si = inv s
    k  = p <> ht <> si

    x' = x + (k #> y)
    p' = (ident 4 - k <> h) <> p


update (Radar rho phi rho' t) kf =
  KF { kf_x = x'
     , kf_p = p'
     , kf_t = t }
  where
    (x, p) = predict (Radar rho phi rho' t) kf

    -- Calculate the Jacobian matrix values 
    px = x ! 0
    py = x ! 1
    vx = x ! 2
    vy = x ! 3

    c1 = px ** 2 + py ** 2
    c2 = sqrt c1
    c3 = c1 * c2

    zRho  = c2;
    zPhi  = (atan2 py px)
    zRho' = (px*vx + py*vy) / zRho

    h = (3><4) [-- Row 1
                px / c2, py / c2, 0, 0,
                -- Row 2
                -py / c1, px / c1, 0, 0,
                -- Row 3
                py*(vx * py - vy * px)/c3,
                px*(vy * px - vx * py)/c3,
                px/c2,
                py/c2]
    r = (3><3) [0.09, 0.0000, 0.00,
                0.00, 0.0009, 0.00,
                0.00, 0.0000, 0.09]
    z = 3 |> [rho, phi, rho'] - 3 |> [zRho, zPhi, zRho']
    y = 3 |> [z ! 0, atan2 (sin (z ! 1)) (cos (z ! 1)), z ! 2]

    ht = tr h
    s  = h <> p <> ht + r
    si = inv s
    k  = p <> ht <> si

    x' = x + (k #> y)
    p' = (ident 4 - k <> h) <> p
