#include "defs.hpp"

double getWheelBase(double xL, double xR){
  double wheelBase = cambase - xL + xR;
  return wheelBase;
}

double getTrack(zL, zR){
  double track = camTrack - zL - zR;
  return track;
}

double getTow(xL, zL, xR, zR){
    double tow = atan((zR-zL)/(xR - xL));
    return tow;
}

double getCamber(yL, zL, yR, zR){
    double camber = atan((zR-zL)/(yR - yL));
    return camber;
}
