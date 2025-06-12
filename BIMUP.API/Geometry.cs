using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Autodesk.AutoCAD.Geometry;

public static class GeometryUtilities
{
    #region Constants

    private const double DEFAULT_TOLERANCE = 1e-6;

    #endregion Constants
}


public class GeoPlane
{
    #region Private Properties
    private Vector3d _normal;
    private Point3d _origin;
    private Vector3d _xAxis;
    private Vector3d _yAxis;
    #endregion

    #region Public Properties
    public Point3d Origin
    {
        get { return _origin; }
        set { _origin = value; }
    }
    public Vector3d XAxis
    {
        get { return _xAxis; }  
        set { _xAxis = value; }
    }

    public Vector3d YAxis
    {
        get { return _yAxis; }
        set { _yAxis = value; }
    }

    public Vector3d Normal
    {
        get { return _normal; }
        set { _normal = value; }
    }
    #endregion

    #region Construction

    public GeoPlane(Point3d origin,Vector3d normal)
    {
        _origin = origin;
        _normal = normal;
    }

    #endregion
}

public class GeoCoordinateSystem
{
    #region Private Properties
    private Point3d _origin;

    private Vector3d _xAxis;
    private Vector3d _yAxis;
    private Vector3d _zAxis;

    #endregion

    #region Public Properties
    public Point3d Origin { get { return _origin; } }
    public Vector3d XAxis { get { return _xAxis; } }
    public Vector3d YAxis { get { return _yAxis; } }
    public Vector3d ZAxis { get { return _zAxis; } }
    #endregion

    #region Internal Constructor (Hàm khởi tạo nội bộ)
    /// <summary>
    /// Hàm khởi tạo chính, nhận đầy đủ thông tin. 
    /// Các hàm khác sẽ gọi hàm này sau khi đã tính toán xong.
    /// </summary>
    private GeoCoordinateSystem(Point3d origin, Vector3d xAxis, Vector3d yAxis, Vector3d zAxis)
    {
        _origin = origin;
        // Đảm bảo các vector trục luôn là vector đơn vị (chiều dài = 1)
        _xAxis = xAxis.GetNormal();
        _yAxis = yAxis.GetNormal();
        _zAxis = zAxis.GetNormal();
    }
    #endregion

    #region Identity (Hệ tọa độ gốc WCS)
    // Tương ứng với node "Identity"

    /// <summary>
    /// Tạo một hệ tọa độ trùng với Hệ tọa độ Thế giới (World Coordinate System - WCS).
    /// </summary>
    public GeoCoordinateSystem()
    {
        _origin = Point3d.Origin; // Gốc tại (0,0,0)
        _xAxis = Vector3d.XAxis;  // (1,0,0)
        _yAxis = Vector3d.YAxis;  // (0,1,0)
        _zAxis = Vector3d.ZAxis;  // (0,0,1)
    }

    /// <summary>
    /// Một thuộc tính tĩnh (static) để lấy nhanh hệ tọa độ WCS mà không cần tạo đối tượng mới.
    /// </summary>
    public static GeoCoordinateSystem WCS
    {
        get { return new GeoCoordinateSystem(); }
    }
    #endregion

    #region Construction
    public GeoCoordinateSystem(double x, double y) : this()
    {
        _origin = new Point3d(x, y, 0);
    }
    public GeoCoordinateSystem(double x, double y,double z): this()
    {
        _origin = new Point3d(x, y, z);
    }

    #endregion
}

