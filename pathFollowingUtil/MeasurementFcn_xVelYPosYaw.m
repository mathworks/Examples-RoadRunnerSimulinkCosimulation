function yk = MeasurementFcn_xVelYPosYaw(xk, offset)
yk = xk([1, 6, 4, 5]+offset);
end