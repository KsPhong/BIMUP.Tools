using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

public static class GeometryUtils
{
    public static double NormalizeAngle(double angle)
    {
        while (angle < 0) angle += 2 * Math.PI;
        while (angle >= 2 * Math.PI) angle -= 2 * Math.PI;
        return angle;
    }

    public static bool IsAngleBetween(double angle, double start, double end)
    {
        if (start < end)
            return angle > start && angle < end;
        else
            return angle > start || angle < end;
    }
}
