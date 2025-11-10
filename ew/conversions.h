#pragma once

#include <raylib.h>
#include <cyclone/cyclone.h>

namespace ew {
    /// <summary>
    /// Converts a Cyclone Matrix4 to Raylib Matrix
    /// </summary>
    /// <param name="m"></param>
    /// <returns></returns>
    inline Matrix CTR(const cyclone::Matrix4& m) {
        Matrix newMat;
        //First row
        newMat.m0 = m.data[0];
        newMat.m4 = m.data[1];
        newMat.m8 = m.data[2];
        newMat.m12 = m.data[3];
        //Second row
        newMat.m1 = m.data[4];
        newMat.m5 = m.data[5];
        newMat.m9 = m.data[6];
        newMat.m13 = m.data[7];
        //Third row
        newMat.m2 = m.data[8];
        newMat.m6 = m.data[9];
        newMat.m10 = m.data[10];
        newMat.m14 = m.data[11];
        //Fourth row
        newMat.m3 = 0;
        newMat.m7 = 0;
        newMat.m11 = 0;
        newMat.m15 = 1;
        return newMat;
    }
    /// <summary>
    /// Converts a Raylib Matrix to Cyclone Matrix
    /// </summary>
    /// <param name="m"></param>
    /// <returns></returns>
    inline cyclone::Matrix4 RTC(const Matrix& m) {
        cyclone::Matrix4 newMat;
        //First row
        newMat.data[0] = m.m0;
        newMat.data[1] = m.m4;
        newMat.data[2] = m.m8;
        newMat.data[3] = m.m12;
        //Second row
        newMat.data[4] = m.m1;
        newMat.data[5] = m.m5;
        newMat.data[6] = m.m9;
        newMat.data[7] = m.m13;
        //Third row
        newMat.data[8] = m.m2;
        newMat.data[9] = m.m6;
        newMat.data[10] = m.m10;
        newMat.data[11] = m.m14;
        //Fourth row is assumed to be 0,0,0,1
        return newMat;
    }
    /// <summary>
    /// Converts a Raylib Vector3 to Cyclone Vector3
    /// </summary>
    /// <param name="v"></param>
    /// <returns></returns>
    inline cyclone::Vector3 RTC(const Vector3 v) {
        return {v.x, v.y, v.z};
    }
    /// <summary>
   /// Converts a Cyclone Vector3 to Raylib Vector3
   /// </summary>
   /// <param name="v"></param>
   /// <returns></returns>
    inline Vector3 CTR(const cyclone::Vector3 v) {
        return Vector3{ v.x, v.y, v.z };
    }
}