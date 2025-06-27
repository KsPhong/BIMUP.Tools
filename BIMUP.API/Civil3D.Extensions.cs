using Autodesk.AutoCAD.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace BIMUP.API
{
    public static class Point3dExtensions
    {
        /// <summary>
        /// Tính khoảng cách đến một GeoCurves.
        /// </summary>
        public static double DistanceTo(this Point3d point, GeoCurves curve)
        {
            return curve.DistanceTo(point);
        }
    }
}
