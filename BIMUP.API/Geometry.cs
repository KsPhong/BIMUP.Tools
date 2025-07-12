using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Autodesk.AutoCAD.ApplicationServices;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.EditorInput;
using Autodesk.AutoCAD.Geometry;

public static class GeometryUtils
{
    /// <summary>
    /// Compares two 3D points with default tolerance.
    /// </summary>
    public static bool Equals(Point3d p1, Point3d p2)
    {
        return p1.IsEqualTo(p2, new Tolerance(Constants.DefaultTolerance, Constants.DefaultTolerance));
    }

    /// <summary>
    /// Removes duplicate 3D points from the list.
    /// </summary>
    public static List<Point3d> OverKillPoint3d(List<Point3d> points)
    {
        if (points == null || points.Count == 0) return new List<Point3d>();

        var sorted = points.OrderBy(p => p.X).ThenBy(p => p.Y).ThenBy(p => p.Z).ToList();
        var result = new List<Point3d>();
        Point3d? last = null;

        foreach (var pt in sorted)
        {
            if (last == null || !Equals(pt, last.Value))
            {
                result.Add(pt);
                last = pt;
            }
        }

        return result;
    }

    /// <summary>
    /// Removes overlapping or duplicate GeoLine segments from a list of curves (GeoLine or GeoPolycurve).
    /// </summary>
    public static List<GeoLine> OverKill(List<GeoCurves> input)
    {
        var allLines = new List<GeoLine>();
        foreach (var curve in input)
        {
            if (curve is GeoLine l) allLines.Add(l);
            else if (curve is GeoPolycurve poly) allLines.AddRange(poly.Segments.OfType<GeoLine>());
        }

        var groups = new List<List<GeoLine>>();
        foreach (var line in allLines)
        {
            var dir = line.Direction.GetNormal();
            if (dir.X < 0 || (Math.Abs(dir.X) < 1e-6 && dir.Y < 0) ||
                (Math.Abs(dir.X) < 1e-6 && Math.Abs(dir.Y) < 1e-6 && dir.Z < 0))
                dir = -dir;

            bool added = false;
            foreach (var group in groups)
            {
                var rep = group[0];
                var repDir = rep.Direction.GetNormal();
                if (repDir.X < 0 || (Math.Abs(repDir.X) < 1e-6 && repDir.Y < 0) ||
                    (Math.Abs(repDir.X) < 1e-6 && Math.Abs(repDir.Y) < 1e-6 && repDir.Z < 0))
                    repDir = -repDir;

                // Same direction
                if (!repDir.IsParallelTo(dir, new Tolerance(1e-6, 1e-6)))
                    continue;

                // On same line
                var v = line.StartPoint - rep.StartPoint;
                if (repDir.CrossProduct(v).Length < 1e-6)
                {
                    group.Add(line);
                    added = true;
                    break;
                }
            }

            if (!added)
                groups.Add(new List<GeoLine> { line });
        }

        var result = new List<GeoLine>();
        foreach (var group in groups)
        {
            if (group.Count == 0) continue;

            var dir = group[0].Direction.GetNormal();
            if (dir.X < 0 || (Math.Abs(dir.X) < 1e-6 && dir.Y < 0) ||
                (Math.Abs(dir.X) < 1e-6 && Math.Abs(dir.Y) < 1e-6 && dir.Z < 0))
                dir = -dir;

            var origin = group[0].StartPoint;

            // Collect all start and end points
            var allPts = new List<Point3d>();
            foreach (var l in group)
            {
                allPts.Add(l.StartPoint);
                allPts.Add(l.EndPoint);
            }

            // Remove duplicates
            if (allPts == null || allPts.Count == 0) continue;
            var sorted = allPts.OrderBy(p => p.X).ThenBy(p => p.Y).ThenBy(p => p.Z).ToList();
            var cleanPts = new List<Point3d>();
            Point3d? last = null;
            foreach (var pt in sorted)
            {
                if (last == null || !pt.IsEqualTo(last.Value, new Tolerance(Constants.DefaultTolerance, Constants.DefaultTolerance)))
                {
                    cleanPts.Add(pt);
                    last = pt;
                }
            }

            // Sort along direction
            cleanPts.Sort((p1, p2) =>
            {
                double t1 = (p1 - origin).DotProduct(dir);
                double t2 = (p2 - origin).DotProduct(dir);
                return t1.CompareTo(t2);
            });

            // Generate unique segments
            for (int i = 0; i < cleanPts.Count - 1; i++)
            {
                var p1 = cleanPts[i];
                var p2 = cleanPts[i + 1];
                if (p1.DistanceTo(p2) < Constants.DefaultTolerance) continue;

                var mid = new Point3d(
                    (p1.X + p2.X) / 2.0,
                    (p1.Y + p2.Y) / 2.0,
                    (p1.Z + p2.Z) / 2.0
                );

                bool matched = false;
                foreach (var g in group)
                {
                    if (g.DistanceTo(mid) < Constants.DefaultTolerance)
                    {
                        matched = true;
                        break;
                    }
                }

                if (matched)
                    result.Add(new GeoLine(p1, p2));
            }
        }

        return result;
    }

}

public static class Constants
{
    /// <summary>
    /// Default numerical tolerance used for geometric comparisons.
    /// </summary>
    public const double DefaultTolerance = 1e-9;
}

public class GeoPlane
{
    #region Properties

    private Point3d _origin;       // Plane origin
    private Vector3d _normal;      // Normal vector
    private Vector3d _xAxis;       // Optional X axis
    private Vector3d _yAxis;       // Optional Y axis

    #endregion

    #region Public Properties

    public Point3d Origin { get => _origin; set => _origin = value; }       // Origin point of the plane
    public Vector3d XAxis { get => _xAxis; set => _xAxis = value; }       // X direction on the plane
    public Vector3d YAxis { get => _yAxis; set => _yAxis = value; }       // Y direction on the plane
    public Vector3d Normal { get => _normal; set => _normal = value; }      // Normal vector of the plane

    #endregion

    #region Construction

    /// <summary>
    /// Initializes a new instance of the <see cref="GeoPlane"/> class with the specified origin and normal vector.
    /// </summary>
    /// <param name="origin">A point on the plane.</param>
    /// <param name="normal">The normal vector of the plane. It does not need to be normalized.</param>
    public GeoPlane(Point3d origin, Vector3d normal)
    {
        _origin = origin;
        _normal = normal;
    }

    /// <summary>
    /// Constructs a plane from three non-collinear points.
    /// </summary>
    /// <param name="p1">First point (used as origin)</param>
    /// <param name="p2">Second point</param>
    /// <param name="p3">Third point</param>
    public GeoPlane(Point3d p1, Point3d p2, Point3d p3)
    {
        _origin = p1;
        _normal = ((p2 - p1).CrossProduct(p3 - p1)).GetNormal(); // Compute normal vector of the plane
    }
    #endregion

    #region Method
    /// <summary>
    /// Returns a unit X axis vector on the plane, orthogonal to the normal.
    /// </summary>
    public Vector3d GetXAxis()
    {
        // Use Z-axis as reference unless the normal is nearly vertical
        Vector3d refVec = Math.Abs(_normal.Z) < 0.99 ? Vector3d.ZAxis : Vector3d.XAxis;
        return _normal.CrossProduct(refVec).GetNormal(); // X axis lies on the plane
    }

    /// <summary>
    /// Projects a 3D point onto the plane's 2D coordinate system (local X and Y).
    /// </summary>
    /// <param name="pt">3D point to project</param>
    /// <returns>Projected 2D coordinates on the plane</returns>
    public Point2d ProjectPointTo2D(Point3d pt)
    {
        var xAxis = GetXAxis();                                 // Local X axis on the plane
        var yAxis = _normal.CrossProduct(xAxis).GetNormal();    // Local Y axis orthogonal to both normal and X

        var v = pt - _origin;                                   // Vector from origin to point
        double x = v.DotProduct(xAxis);                         // Projection onto X
        double y = v.DotProduct(yAxis);                         // Projection onto Y

        return new Point2d(x, y);
    }

    public Entity[] Test()
    {
        // Gốc và vector cơ sở
        var origin = _origin;
        var normal = _normal.GetNormal();

        // Tạo hệ trục XY nằm trên mặt phẳng
        Vector3d xAxis = _xAxis.Length > 0 ? _xAxis.GetNormal() :
                         (Math.Abs(normal.DotProduct(Vector3d.XAxis)) > 0.9 ? Vector3d.YAxis : Vector3d.XAxis).CrossProduct(normal).GetNormal();

        Vector3d yAxis = normal.CrossProduct(xAxis).GetNormal();

        // Tạo 4 điểm vuông góc theo hệ trục mặt phẳng
        double size = 2.5; // nửa cạnh hình vuông (tổng 5x5)
        Point3d p1 = origin + xAxis * -size + yAxis * -size;
        Point3d p2 = origin + xAxis * size + yAxis * -size;
        Point3d p3 = origin + xAxis * size + yAxis * size;
        Point3d p4 = origin + xAxis * -size + yAxis * size;

        // Tạo Polyline kín
        var pl = new Polyline(4);
        pl.AddVertexAt(0, new Point2d(0, 0), 0, 0, 0);
        pl.AddVertexAt(1, new Point2d(5, 0), 0, 0, 0);
        pl.AddVertexAt(2, new Point2d(5, 5), 0, 0, 0);
        pl.AddVertexAt(3, new Point2d(0, 5), 0, 0, 0);
        pl.Closed = true;

        // Di chuyển Polyline từ (0,0,0) sang tọa độ p1 và xoay theo hệ trục mặt phẳng
        var mat = Matrix3d.AlignCoordinateSystem(
            Point3d.Origin, Vector3d.XAxis, Vector3d.YAxis, Vector3d.ZAxis,
            p1, xAxis, yAxis, normal
        );

        var plTransformed = (Entity)pl.Clone();
        plTransformed.TransformBy(mat);

        // Tạo Region từ Polyline
        DBObjectCollection curves = new DBObjectCollection();
        curves.Add(plTransformed);
        var regions = Region.CreateFromCurves(curves);

        // Tạo đường vector pháp tuyến màu đỏ
        var line = new Line(origin, origin + normal * 5);
        line.ColorIndex = 1; // Red

        // Trả về Region + Line
        var entities = new List<Entity>();
        entities.AddRange(regions.Cast<Entity>());
        entities.Add(line);

        return entities.ToArray();
    }
    #endregion
}

public class GeoCoordinateSystem
{
    #region Fields

    private Point3d _origin;       // Coordinate system origin
    private Vector3d _xAxis;       // Local X axis (unit vector)
    private Vector3d _yAxis;       // Local Y axis (unit vector)
    private Vector3d _zAxis;       // Local Z axis (unit vector)

    #endregion

    #region Properties

    public Point3d Origin => _origin;        // Origin point
    public Vector3d XAxis => _xAxis;         // X axis direction
    public Vector3d YAxis => _yAxis;         // Y axis direction
    public Vector3d ZAxis => _zAxis;         // Z axis direction

    #endregion

    #region Constructors

    /// <summary>
    /// Initializes a new coordinate system with full axis definitions.
    /// All axes will be normalized automatically.
    /// </summary>
    /// <param name="origin">The origin point of the coordinate system.</param>
    /// <param name="xAxis">X axis direction.</param>
    /// <param name="yAxis">Y axis direction.</param>
    /// <param name="zAxis">Z axis direction.</param>
    public GeoCoordinateSystem(Point3d origin, Vector3d xAxis, Vector3d yAxis, Vector3d zAxis)
    {
        _origin = origin;
        _xAxis = xAxis.GetNormal();
        _yAxis = yAxis.GetNormal();
        _zAxis = zAxis.GetNormal();
    }

    /// <summary>
    /// Initializes the coordinate system as a standard WCS (0,0,0 + unit axes).
    /// </summary>
    public GeoCoordinateSystem()
    {
        _origin = Point3d.Origin;
        _xAxis = Vector3d.XAxis;
        _yAxis = Vector3d.YAxis;
        _zAxis = Vector3d.ZAxis;
    }

    /// <summary>
    /// Initializes a WCS-based coordinate system with custom XY origin (Z = 0).
    /// </summary>
    public GeoCoordinateSystem(double x, double y) : this()
    {
        _origin = new Point3d(x, y, 0);
    }

    /// <summary>
    /// Initializes a WCS-based coordinate system with custom XYZ origin.
    /// </summary>
    public GeoCoordinateSystem(double x, double y, double z) : this()
    {
        _origin = new Point3d(x, y, z);
    }

    #endregion

    #region Static

    /// <summary>
    /// Gets a new instance of the world coordinate system (WCS).
    /// </summary>
    public static GeoCoordinateSystem WCS => new GeoCoordinateSystem();

    #endregion

    #region Method

    /// <summary>
    /// Rotates the coordinate system around an arbitrary axis by a given angle in degrees.
    /// </summary>
    /// <param name="axis">
    /// The rotation axis as a 3D vector. It does not need to be normalized.
    /// </param>
    /// <param name="angleDegrees">
    /// The rotation angle in degrees. Positive values rotate counterclockwise 
    /// according to the right-hand rule with respect to the axis direction.
    /// </param>
    /// <returns>
    /// A new <see cref="GeoCoordinateSystem"/> instance with X, Y, and Z axes rotated 
    /// around the specified axis by the given angle. The origin remains unchanged.
    /// </returns>
    /// <exception cref="ArgumentException">
    /// Thrown when the input axis has zero length.
    /// </exception>
    public GeoCoordinateSystem Rotate(Vector3d axis, double angleDegrees)
    {
        if (axis.Length < Constants.DefaultTolerance)
            throw new ArgumentException("Rotation axis must be non-zero.");

        double angleRadians = angleDegrees * (Math.PI / 180.0);
        Vector3d axisNorm = axis.GetNormal();

        Vector3d newX = XAxis.TransformBy(Matrix3d.Rotation(angleRadians, axisNorm, Point3d.Origin));
        Vector3d newY = YAxis.TransformBy(Matrix3d.Rotation(angleRadians, axisNorm, Point3d.Origin));
        Vector3d newZ = ZAxis.TransformBy(Matrix3d.Rotation(angleRadians, axisNorm, Point3d.Origin));

        return new GeoCoordinateSystem(Origin, newX, newY, newZ);
    }

    public Entity[] Test()
    {
        double length = 5.0;

        var xLine = new Line(_origin, _origin + _xAxis * length) { ColorIndex = 1 }; // Đỏ
        var yLine = new Line(_origin, _origin + _yAxis * length) { ColorIndex = 3 }; // Xanh lá
        var zLine = new Line(_origin, _origin + _zAxis * length) { ColorIndex = 5 }; // Xanh than

        return new Entity[] { xLine, yLine, zLine };
    }
    #endregion
}

public abstract class GeoCurves
{
    #region Properties

    public abstract Point3d StartPoint { get; protected set; }   // Start point of the curve
    public abstract Point3d EndPoint { get; protected set; }   // End point of the curve
    public abstract bool IsClosed { get; protected set; }   // Whether the curve is closed
    public abstract double Length { get; protected set; }   // Total length of the curve

    #endregion

    #region Abstract Methods
    /// <summary>
    /// Tính tham số t (0 ≤ t ≤ 1) của một điểm bất kỳ so với GeoLine,
    /// nhưng chỉ xét trên mặt phẳng OXY (Z = 0), sau đó có thể dùng t để tìm vị trí 3D.
    /// </summary>
    /// <param name="point">Điểm cần tính t (có thể nằm ngoài đường)</param>
    /// <returns>Giá trị t đã clamp từ 0 đến 1</returns>
    public abstract double ParameterAtPoint2D(Point3d point);

    /// <summary>
    /// Returns the local coordinate system at a parameter t (from 0 to 1).
    /// Typically used to define perpendicular cross-sections along the curve.
    /// </summary>
    public abstract GeoCoordinateSystem CoordinateSystemAtParameter(double t);

    /// <summary>
    /// Returns the coordinate system at a specific distance from the start point (absolute length).
    /// </summary>
    public abstract GeoCoordinateSystem CoordinateSystemAtSegmentLength(double t);

    /// <summary>
    /// Gets the normal vector at parameter t.
    /// Useful for constructing perpendicular planes or visual effects.
    /// </summary>
    public abstract Vector3d NormalAtParameter(double t);

    /// <summary>
    /// Calculates parameter t (0 ≤ t ≤ 1) for a given point projected onto the curve.
    /// </summary>
    public abstract double ParameterAtPoint(Point3d point);

    /// <summary>
    /// Returns a perpendicular plane at parameter t on the curve.
    /// </summary>
    public abstract GeoPlane PlaneAtParameter(double t);

    /// <summary>
    /// Gets the point on the curve corresponding to parameter t.
    /// </summary>
    public abstract Point3d PointAtParameter(double t);

    /// <summary>
    /// Projects the entire curve onto a given plane.
    /// </summary>
    public abstract GeoCurves PullOntoPlane(GeoPlane plane);

    /// <summary>
    /// Reverses the direction of the curve (start becomes end and vice versa).
    /// </summary>
    public abstract void Reverse();

    /// <summary>
    /// Splits the curve at specified parameters (0 ≤ t ≤ 1).
    /// </summary>
    public abstract List<GeoCurves> SplitByParameter(List<double> parameters);

    /// <summary>
    /// Splits the curve at specific points (projected onto the curve).
    /// </summary>
    public abstract List<GeoCurves> SplitByPoints(List<Point3d> points);

    /// <summary>
    /// Computes the shortest distance from a point to the curve.
    /// </summary>
    public abstract double DistanceTo(Point3d point);

    /// <summary>
    /// Calculates the intersection between this curve and another GeoCurves object.
    /// </summary>
    public abstract IntersectionResult IntersectWith(GeoCurves other);

    /// <summary>
    /// Returns the local coordinate system at the specified parameter t,
    /// with the Z axis fixed (typically upward or normal direction).
    /// </summary>
    /// <param name="t">Curve parameter (usually between 0 and 1)</param>
    /// <returns>Local coordinate system at parameter t</returns>
    public abstract GeoCoordinateSystem CoordinateSystemAtParameterFixZ(double t);

    public abstract Entity[] Test();

    #endregion
}

public sealed class GeoLine: GeoCurves
{
    #region Fields

    private Point3d _startPoint;     // Start point
    private Point3d _endPoint;       // End point
    private Vector3d _direction;     // Normalized direction vector
    private bool m_isClosed;         // Always false for line
    private double _length;          // Length of the line

    #endregion

    #region Properties

    public override Point3d StartPoint { get => _startPoint; protected set => _startPoint = value; }
    public override Point3d EndPoint { get => _endPoint; protected set => _endPoint = value; }
    public override bool IsClosed { get => m_isClosed; protected set => m_isClosed = value; }
    public override double Length { get => _length; protected set => _length = value; }

    public Vector3d Direction => _direction; // Unit direction from start to end

    #endregion

    #region Constructors

    /// <summary>
    /// Constructs a GeoLine from two 3D points.
    /// </summary>
    /// <param name="start">Start point of the line</param>
    /// <param name="end">End point of the line</param>
    public GeoLine(Point3d start, Point3d end)
    {
        _startPoint = start;
        _endPoint = end;
        _direction = (end - start).GetNormal();   // Calculate unit direction vector
        _length = start.DistanceTo(end);          // Calculate line length
    }

    /// <summary>
    /// Constructs a GeoLine from an AutoCAD Line entity.
    /// </summary>
    /// <param name="line">AutoCAD Line entity</param>
    public GeoLine(Line line)
    {
        _startPoint = line.StartPoint;
        _endPoint = line.EndPoint;
        _direction = (_endPoint - _startPoint).GetNormal();  // Calculate unit direction vector
        _length = _startPoint.DistanceTo(_endPoint);         // Calculate line length
    }
    #endregion

    #region Override
    /// <summary>
    /// Gets the point on the line at parameter t (0 ≤ t ≤ 1).
    /// </summary>
    /// <param name="t">Normalized parameter (0 = start, 1 = end).</param>
    /// <returns>The corresponding point on the line.</returns>
    public override Point3d PointAtParameter(double t)
    {
        return StartPoint + t * (EndPoint - StartPoint);
    }

    /// <summary>
    /// Calculates the parameter t (0 ≤ t ≤ 1) for a given point projected onto the line.
    /// </summary>
    /// <param name="point">The 3D point to project onto the line.</param>
    /// <returns>The parameter t closest to the point.</returns>
    public override double ParameterAtPoint(Point3d point)
    {
        var lineVec = EndPoint - StartPoint;
        var toPoint = point - StartPoint;

        // Trường hợp đoạn thẳng có chiều dài gần bằng không
        if (lineVec.LengthSqrd < Constants.DefaultTolerance)
        {
            return 0.0;
        }

        // Tính toán tham số t của hình chiếu vuông góc lên đường thẳng vô hạn
        double t = toPoint.DotProduct(lineVec) / lineVec.LengthSqrd;

        // "Kẹp" (clamp) giá trị t vào khoảng [0, 1]
        // Điều này đảm bảo điểm tương ứng luôn nằm trên đoạn thẳng.
        return Math.Max(0, Math.Min(1, t));
    }

    /// <summary>
    /// Tính tham số t (0 ≤ t ≤ 1) của một điểm bất kỳ so với GeoLine,
    /// nhưng chỉ xét trên mặt phẳng OXY (Z = 0), sau đó có thể dùng t để tìm vị trí 3D.
    /// </summary>
    /// <param name="point">Điểm cần tính t (có thể nằm ngoài đường)</param>
    /// <returns>Giá trị t đã clamp từ 0 đến 1</returns>
    public override double ParameterAtPoint2D(Point3d point)
    {
        // 1. Chiếu điểm đầu và cuối của GeoLine xuống mặt phẳng OXY (Z = 0)
        Point3d p0 = new Point3d(StartPoint.X, StartPoint.Y, 0);
        Point3d p1 = new Point3d(EndPoint.X, EndPoint.Y, 0);

        // 2. Chiếu điểm cần kiểm tra xuống OXY
        Point3d pTest = new Point3d(point.X, point.Y, 0);

        // 3. Tính vector của đoạn thẳng (chiếu xuống OXY) và vector từ p0 đến điểm cần kiểm tra
        Vector3d lineVec = p1 - p0;
        Vector3d toPoint = pTest - p0;

        // 4. Tính t theo công thức nội suy: (toPoint ⋅ lineVec) / |lineVec|²
        double t = toPoint.DotProduct(lineVec) / lineVec.LengthSqrd;

        // 5. Giới hạn t trong khoảng [0, 1] để tránh vượt khỏi đoạn gốc
        if (t < 0) t = 0;
        if (t > 1) t = 1;

        return t;
    }

    /// <summary>
    /// Returns the local coordinate system at parameter t along the line,
    /// where:
    /// - Y-axis is the line's direction (forward),
    /// - Z-axis is global Z if not parallel to Y, otherwise uses fallback,
    /// - X-axis is computed as Y × Z (to the right),
    /// and Z is re-adjusted as X × Y to ensure orthonormal system.
    /// </summary>
    public override GeoCoordinateSystem CoordinateSystemAtParameter(double t)
    {
        var origin = PointAtParameter(t);
        Vector3d y = Direction.GetNormal();

        // Choose Z: global Z if not nearly parallel to Y, fallback if too close
        Vector3d z = y.CrossProduct(Vector3d.ZAxis).Length > Constants.DefaultTolerance
            ? Vector3d.ZAxis
            : Vector3d.YAxis;

        // Compute X to the right of Y
        Vector3d x = y.CrossProduct(z).GetNormal();

        // Recompute Z to enforce strict orthonormal right-handed system
        z = x.CrossProduct(y).GetNormal();

        return new GeoCoordinateSystem(origin, x, y, z);
    }

    /// <summary>
    /// Returns a local coordinate system at parameter t along the line,
    /// with Z-axis fixed as vertical (global Z). 
    /// - Y-axis follows the line's direction,
    /// - X-axis is Y × Z (to the right),
    /// and Z is recomputed from X × Y to enforce orthonormality.
    /// </summary>
    public override GeoCoordinateSystem CoordinateSystemAtParameterFixZ(double t)
    {
        var origin = PointAtParameter(t);
        Vector3d y = Direction.GetNormal();
        Vector3d z = Vector3d.ZAxis;

        Vector3d x = y.CrossProduct(z).GetNormal();

        // Handle degenerate case where Y is parallel to Z (vertical line)
        if (x.Length < Constants.DefaultTolerance)
            x = Vector3d.XAxis;

        // Recompute Z to enforce right-handed coordinate system
        z = x.CrossProduct(y).GetNormal();

        return new GeoCoordinateSystem(origin, x, y, z);
    }

    /// <summary>
    /// Returns the coordinate system at a specific length from the start point (absolute distance).
    /// </summary>
    /// <param name="t">Distance in units from the start point.</param>
    public override GeoCoordinateSystem CoordinateSystemAtSegmentLength(double t)
    {
        double param = t / Length;
        return CoordinateSystemAtParameter(param);
    }

    /// <summary>
    /// Gets the vector normal to the line at parameter t.
    /// The normal is perpendicular to the line and not aligned with world Z-axis.
    /// </summary>
    public override Vector3d NormalAtParameter(double t)
    {
        Vector3d worldZ = Vector3d.ZAxis;

        if (Math.Abs(Direction.DotProduct(worldZ)) > 0.99)
            worldZ = Vector3d.YAxis;

        return Direction.CrossProduct(worldZ).GetNormal();
    }

    /// <summary>
    /// Creates a plane perpendicular to the line at parameter t.
    /// </summary>
    public override GeoPlane PlaneAtParameter(double t)
    {
        var origin = PointAtParameter(t);
        var normal = NormalAtParameter(t);
        return new GeoPlane(origin, Direction);
    }

    /// <summary>
    /// Projects the entire line onto the specified plane.
    /// </summary>
    /// <param name="plane">The target plane to project onto.</param>
    public override GeoCurves PullOntoPlane(GeoPlane plane)
    {
        Vector3d n = plane.Normal;

        Point3d project(Point3d pt)
        {
            double d = (pt - plane.Origin).DotProduct(n);
            return pt - d * n;
        }

        return new GeoLine(project(StartPoint), project(EndPoint));
    }

    /// <summary>
    /// Reverses the direction of the line (start ↔ end).
    /// </summary>
    public override void Reverse()
    {
        var temp = _startPoint;
        _startPoint = _endPoint;
        _endPoint = temp;

        _direction = (_endPoint - _startPoint).GetNormal();
    }

    /// <summary>
    /// Splits the line at specified normalized parameters (t values from 0 to 1).
    /// </summary>
    /// <param name="parameters">List of t values to split at.</param>
    public override List<GeoCurves> SplitByParameter(List<double> parameters)
    {
        var result = new List<GeoCurves>();

        if (parameters == null || parameters.Count == 0)
        {
            result.Add(this);
            return result;
        }

        var sortedParams = parameters
            .Select(p => p < 0 ? 0 : (p > 1 ? 1 : p))
            .Distinct()
            .OrderBy(p => p)
            .ToList();

        double prevT = 0;

        foreach (var t in sortedParams)
        {
            if (Math.Abs(t - prevT) < 1e-6) continue;

            Point3d pt1 = PointAtParameter(prevT);
            Point3d pt2 = PointAtParameter(t);
            result.Add(new GeoLine(pt1, pt2));

            prevT = t;
        }

        if (prevT < 1.0 - 1e-6)
        {
            Point3d pt1 = PointAtParameter(prevT);
            Point3d pt2 = PointAtParameter(1.0);
            result.Add(new GeoLine(pt1, pt2));
        }

        return result;
    }

    /// <summary>
    /// Splits the line at the closest projection points of the given 3D points.
    /// </summary>
    /// <param name="points">3D points to project onto the line and split at.</param>
    public override List<GeoCurves> SplitByPoints(List<Point3d> points)
    {
        if (points == null || points.Count == 0)
            return new List<GeoCurves> { this };

        var parameters = points
            .Select(p => ParameterAtPoint(p))
            .ToList();

        return SplitByParameter(parameters);
    }

    /// <summary>
    /// Computes the shortest distance from a given point to the line segment.
    /// </summary>
    public override double DistanceTo(Point3d point)
    {
        Vector3d v = EndPoint - StartPoint;
        Vector3d w = point - StartPoint;

        double proj = w.DotProduct(v) / v.LengthSqrd;

        // Clamp to segment bounds
        if (proj < 0) proj = 0;
        else if (proj > 1) proj = 1;

        Point3d closest = StartPoint + proj * v;
        return point.DistanceTo(closest);
    }

    /// <summary>
    /// Computes the intersection with another GeoCurves object.
    /// </summary>
    /// <param name="other">The other curve to intersect with.</param>
    public override IntersectionResult IntersectWith(GeoCurves other)
    {
        if (other is GeoLine line)
            return IntersectWithLine(line);
        else if (other is GeoArc arc)
            return IntersectWithArc(arc);
        else if (other is GeoPolycurve poly)
            return IntersectWithPolycurve(poly);
        else
            throw new NotSupportedException($"GeoLine does not support intersection with {other.GetType().Name}");
    }

    public override Entity[] Test()
    {
        var line = new Line(StartPoint, EndPoint)
        {
            ColorIndex = 4 // Màu xanh dương nhạt
        };
        return new Entity[] { line };
    }
    #endregion

    #region Private Function

    /// <summary>
    /// Calculates the signed angle (in radians) between two vectors on a plane.
    /// </summary>
    private double GetAngleOnPlane(Vector3d from, Vector3d to, Vector3d normal)
    {
        double angle = from.GetAngleTo(to);
        return from.CrossProduct(to).DotProduct(normal) >= 0 ? angle : 2 * Math.PI - angle;
    }

    /// <summary>
    /// Computes intersection between this GeoLine and a GeoArc.
    /// </summary>
    public IntersectionResult IntersectWithArc(GeoArc arc)
    {
        Editor ed = Application.DocumentManager.MdiActiveDocument.Editor;

        // 1. Kiểm tra đồng phẳng
        double d1 = (_startPoint - arc.CenterPoint).DotProduct(arc.Normal);
        double d2 = (_endPoint - arc.CenterPoint).DotProduct(arc.Normal);
        if (Math.Abs(d1) > 1e-6 || Math.Abs(d2) > 1e-6)
        {
            ed.WriteMessage("\n❌ Line và Arc không đồng phẳng.");
            return IntersectionResult.None();
        }

        ed.WriteMessage("\n✅ Line và Arc đồng phẳng.");

        // 2. Thiết lập hệ trục cục bộ của Arc
        Vector3d xAxis = (arc.StartPoint - arc.CenterPoint).GetNormal();
        Vector3d yAxis = arc.Normal.CrossProduct(xAxis).GetNormal();

        // 3. Đưa line về 2D
        Vector3d v1 = _startPoint - arc.CenterPoint;
        Vector3d v2 = _endPoint - arc.CenterPoint;
        Point2d p1 = new Point2d(v1.DotProduct(xAxis), v1.DotProduct(yAxis));
        Point2d p2 = new Point2d(v2.DotProduct(xAxis), v2.DotProduct(yAxis));
        Vector2d d = p2 - p1;

        // 4. Phương trình bậc 2
        double a = d.X * d.X + d.Y * d.Y;
        double b = 2 * (p1.X * d.X + p1.Y * d.Y);
        double c = p1.X * p1.X + p1.Y * p1.Y - arc.Radius * arc.Radius;
        double delta = b * b - 4 * a * c;

        ed.WriteMessage(string.Format("\n🧮 a={0}, b={1}, c={2}, delta={3}", a, b, c, delta));

        if (delta < -1e-6)
        {
            ed.WriteMessage("\n❌ Không có giao điểm thực.");
            return IntersectionResult.None();
        }

        delta = Math.Max(0, delta);
        double sqrtDelta = Math.Sqrt(delta);
        double t1 = (-b + sqrtDelta) / (2 * a);
        double t2 = (-b - sqrtDelta) / (2 * a);

        List<double> tList = new List<double>();
        if (t1 >= 0 && t1 <= 1) tList.Add(t1);
        if (t2 >= 0 && t2 <= 1 && Math.Abs(t2 - t1) > 1e-6) tList.Add(t2);

        List<Point3d> intersections = new List<Point3d>();

        foreach (double t in tList)
        {
            double x = p1.X + t * d.X;
            double y = p1.Y + t * d.Y;
            Point3d pt3D = arc.CenterPoint + xAxis * x + yAxis * y;

            Vector3d vecToPt = pt3D - arc.CenterPoint;
            Vector3d vecStart = arc.StartPoint - arc.CenterPoint;
            Vector3d vecEnd = arc.EndPoint - arc.CenterPoint;

            // Tính góc giữa vecStart và vecToPt
            double dot = vecStart.GetNormal().DotProduct(vecToPt.GetNormal());
            dot = Math.Max(-1.0, Math.Min(1.0, dot));
            double angle = Math.Acos(dot);

            // Xác định chiều quay
            double crossZ = vecStart.CrossProduct(vecToPt).DotProduct(arc.Normal);
            if (crossZ < 0)
                angle = 2 * Math.PI - angle;

            // Tính tổng góc cung
            double totalDot = vecStart.GetNormal().DotProduct(vecEnd.GetNormal());
            totalDot = Math.Max(-1.0, Math.Min(1.0, totalDot));
            double totalAngle = Math.Acos(totalDot);
            double totalCrossZ = vecStart.CrossProduct(vecEnd).DotProduct(arc.Normal);
            if (totalCrossZ < 0)
                totalAngle = 2 * Math.PI - totalAngle;

            ed.WriteMessage(
                string.Format("\n🟡 Giao tại: {0}, góc: {1}°, tổng cung: {2}°",
                pt3D,
                angle * 180.0 / Math.PI,
                totalAngle * 180.0 / Math.PI)
            );

            if (angle >= 0 && angle <= totalAngle)
            {
                ed.WriteMessage("\n✅ Giao điểm nằm trong cung.");
                intersections.Add(pt3D);
            }
            else
            {
                ed.WriteMessage("\n⚠️ Giao điểm nằm ngoài cung.");
            }
        }

        if (intersections.Count == 0)
            return IntersectionResult.None();
        else
            return IntersectionResult.Intersect(intersections);
    }

    /// <summary>
    /// Computes intersection between this GeoLine and a GeoPolycurve.
    /// </summary>
    private IntersectionResult IntersectWithPolycurve(GeoPolycurve poly)
    {
        if (poly == null || poly.Segments == null || poly.Segments.Count == 0)
            return IntersectionResult.None();

        List<Point3d> points = new List<Point3d>();
        List<GeoCurves> overlaps = new List<GeoCurves>();

        foreach (GeoCurves segment in poly.Segments)
        {
            IntersectionResult result = this.IntersectWith(segment);

            if (result == null || result.Type == IntersectionResult.IntersectType.None)
                continue;

            if (result.Type == IntersectionResult.IntersectType.Intersect && result.Points != null)
            {
                foreach (Point3d pt in result.Points)
                {
                    if (!points.Any(p => p.DistanceTo(pt) < Constants.DefaultTolerance))
                        points.Add(pt);
                }
            }

            if ((result.Type == IntersectionResult.IntersectType.Overlap ||
                 result.Type == IntersectionResult.IntersectType.SameLine)
                && result.OverlapSegments != null)
            {
                overlaps.AddRange(result.OverlapSegments);
            }
        }

        if (overlaps.Count > 0)
            return IntersectionResult.Overlap(overlaps);

        if (points.Count > 0)
            return IntersectionResult.Intersect(points);

        return IntersectionResult.None();
    }

    /// <summary>
    /// Computes intersection between this GeoLine and another GeoLine.
    /// Handles parallel, colinear, overlap, and intersection cases.
    /// </summary>
    private IntersectionResult IntersectWithLine(GeoLine other)
    {
        var p1 = this.StartPoint;
        var p2 = this.EndPoint;
        var p3 = other.StartPoint;
        var p4 = other.EndPoint;

        var d1 = p2 - p1;
        var d2 = p4 - p3;

        var cross = d1.CrossProduct(d2);

        // Check for parallelism
        if (cross.Length < Constants.DefaultTolerance)
        {
            var offset = p3 - p1;
            if (offset.CrossProduct(d1).Length < Constants.DefaultTolerance)
            {
                double t1 = offset.DotProduct(d1) / d1.LengthSqrd;
                double t2 = (p4 - p1).DotProduct(d1) / d1.LengthSqrd;

                double tMin = Math.Max(0, Math.Min(t1, t2));
                double tMax = Math.Min(1, Math.Max(t1, t2));

                if (tMin <= tMax)
                {
                    Point3d q1 = p1 + d1.MultiplyBy(tMin);
                    Point3d q2 = p1 + d1.MultiplyBy(tMax);
                    return IntersectionResult.Overlap(new[] { new GeoLine(q1, q2) });
                }

                return IntersectionResult.SameLine(new[] { this });
            }

            return IntersectionResult.None(); // Parallel but not on same line
        }

        // Not parallel → compute closest approach
        var w = p1 - p3;

        double a = d1.DotProduct(d1);
        double b = d1.DotProduct(d2);
        double c = d2.DotProduct(d2);
        double d = d1.DotProduct(w);
        double e = d2.DotProduct(w);
        double denom = a * c - b * b;

        if (Math.Abs(denom) < Constants.DefaultTolerance)
            return IntersectionResult.None(); // Degenerate

        double sc = (b * e - c * d) / denom;
        double tc = (a * e - b * d) / denom;

        // Check if within segment bounds
        if (sc < -Constants.DefaultTolerance || sc > 1 + Constants.DefaultTolerance ||
            tc < -Constants.DefaultTolerance || tc > 1 + Constants.DefaultTolerance)
            return IntersectionResult.None();

        Point3d pt1 = p1 + sc * d1;
        Point3d pt2 = p3 + tc * d2;

        return pt1.DistanceTo(pt2) < Constants.DefaultTolerance
            ? IntersectionResult.Intersect(pt1)
            : IntersectionResult.None();
    }

    #endregion

    #region Methods
    /// <summary>
    /// Computes the shortest distance between this GeoLine and another GeoLine.
    /// </summary>
    /// <param name="other">The other GeoLine</param>
    /// <returns>Minimum distance between two line segments</returns>
    public double DistanceTo(GeoLine other)
    {
        var p1 = this.StartPoint;
        var p2 = this.EndPoint;
        var q1 = other.StartPoint;
        var q2 = other.EndPoint;

        var u = p2 - p1;
        var v = q2 - q1;
        var w0 = p1 - q1;

        double a = u.DotProduct(u);  // u ⋅ u
        double b = u.DotProduct(v);  // u ⋅ v
        double c = v.DotProduct(v);  // v ⋅ v
        double d = u.DotProduct(w0); // u ⋅ (p1 - q1)
        double e = v.DotProduct(w0); // v ⋅ (p1 - q1)

        double denom = a * c - b * b;
        double s, t;

        if (Math.Abs(denom) < 1e-8)
        {
            // Lines are nearly parallel → project q1 onto this line
            s = 0;
            t = (b > c ? d / b : e / c);
        }
        else
        {
            s = (b * e - c * d) / denom;
            t = (a * e - b * d) / denom;
        }

        // Clamp s, t to [0, 1] to stay within the segments
        s = Math.Max(0, Math.Min(1, s));
        t = Math.Max(0, Math.Min(1, t));

        Point3d pointOnThis = p1 + s * u;
        Point3d pointOnOther = q1 + t * v;

        return pointOnThis.DistanceTo(pointOnOther);
    }

    /// <summary>
    /// Returns a formatted string containing geometry info of the GeoLine.
    /// </summary>
    /// <returns>Human-readable information string</returns>
    public string GetInfoString()
    {
        var sb = new StringBuilder();

        sb.AppendLine("===== GeoLine Geometry Info =====");
        sb.AppendLine($"Start X\t{StartPoint.X:0.####}");
        sb.AppendLine($"Start Y\t{StartPoint.Y:0.####}");
        sb.AppendLine($"Start Z\t{StartPoint.Z:0.####}");
        sb.AppendLine($"End X\t{EndPoint.X:0.####}");
        sb.AppendLine($"End Y\t{EndPoint.Y:0.####}");
        sb.AppendLine($"End Z\t{EndPoint.Z:0.####}");

        var dir = EndPoint - StartPoint;
        double length = dir.Length;
        var unitDir = dir.GetNormal();

        sb.AppendLine($"Length\t{length:0.####}");
        sb.AppendLine($"Direction X\t{unitDir.X:0.####}");
        sb.AppendLine($"Direction Y\t{unitDir.Y:0.####}");
        sb.AppendLine($"Direction Z\t{unitDir.Z:0.####}");
        sb.AppendLine("===============================");

        return sb.ToString();
    }
    #endregion

}

public sealed class GeoArc : GeoCurves
{
    #region Fields

    private Point3d m_startpoint;     // Start point of the arc
    private Point3d m_endpoint;       // End point of the arc
    private Point3d m_midPoint;       // Midpoint (used to define arc shape)
    private Point3d m_centerPoint;    // Center of the arc

    private Vector3d m_normal;        // Normal vector of the arc's plane
    private GeoPlane m_plane;         // Plane containing the arc

    private bool m_isClosed;          // Whether the arc is closed (usually false)

    private double m_radius;          // Radius of the arc
    private double m_startAngle;      // Angle at the start point (usually 0)
    private double m_endAngle;        // Angle at the end point (usually = sweep angle)
    private double m_angle;           // Total sweep angle (radians)
    private double m_length;          // Arc length

    #endregion

    #region Properties

    public override Point3d StartPoint { get => m_startpoint; protected set => m_startpoint = value; }   // Start point of the arc
    public override Point3d EndPoint { get => m_endpoint; protected set => m_endpoint = value; }     // End point of the arc
    public override bool IsClosed { get => m_isClosed; protected set => m_isClosed = value; }     // Whether the arc is closed
    public override double Length { get => m_length; protected set => m_length = value; }       // Arc length

    public Point3d MidPoint { get => m_midPoint; set => m_midPoint = value; }               // Midpoint (interpolation input)
    public Point3d CenterPoint { get => m_centerPoint; protected set => m_centerPoint = value; }  // Center of the circle
    public GeoPlane Plane { get => m_plane; protected set => m_plane = value; }         // Plane containing the arc
    public Vector3d Normal { get => m_normal; protected set => m_normal = value; }        // Normal vector of the plane
    public double Radius { get => m_radius; protected set => m_radius = value; }        // Radius of the arc
    public double Angle { get => m_angle; protected set => m_angle = value; }         // Sweep angle in radians
    public double StartAngle { get => m_startAngle; protected set => m_startAngle = value; }    // Start angle (usually 0)
    public double EndAngle { get => m_endAngle; protected set => m_endAngle = value; }      // End angle (= Start + sweep)

    #endregion

    #region Constructors

    /// <summary>
    /// Initializes a GeoArc from three points: start, mid (on arc), and end.
    /// </summary>
    /// <param name="startPoint">Start point of the arc.</param>
    /// <param name="midPoint">Any point lying on the arc between start and end.</param>
    /// <param name="endPoint">End point of the arc.</param>
    /// <exception cref="ArgumentException">Thrown if the three points are colinear or invalid.</exception>
    public GeoArc(Point3d A, Point3d B, Point3d C)
    {
        m_startpoint = A;
        m_midPoint = B;
        m_endpoint = C;
        m_isClosed = false;

        if (A.DistanceTo(B) < 1e-9 || B.DistanceTo(C) < 1e-9 || A.DistanceTo(C) < 1e-9)
            throw new ArgumentException("Defining points must be distinct.");

        Vector3d vAB = B - A;
        Vector3d vAC = C - A;
        Vector3d normal = vAB.CrossProduct(vAC);

        if (normal.Length < 1e-9)
            throw new ArgumentException("Defining points are collinear.");

        m_normal = normal.GetNormal();

        Vector3d vBC = C - B;
        Point3d midAB = A + (B - A) * 0.5;
        Point3d midBC = B + (C - B) * 0.5;

        double[,] M = new double[3, 3] {
        { m_normal.X, m_normal.Y, m_normal.Z },
        { vAB.X, vAB.Y, vAB.Z },
        { vBC.X, vBC.Y, vBC.Z }
            };

                double[] V = new double[3] {
            m_normal.DotProduct(A.GetAsVector()),
            vAB.DotProduct(midAB.GetAsVector()),
            vBC.DotProduct(midBC.GetAsVector())
            };

        double detM = M[0, 0] * (M[1, 1] * M[2, 2] - M[1, 2] * M[2, 1]) -
                      M[0, 1] * (M[1, 0] * M[2, 2] - M[1, 2] * M[2, 0]) +
                      M[0, 2] * (M[1, 0] * M[2, 1] - M[1, 1] * M[2, 0]);

        if (Math.Abs(detM) < 1e-9)
            throw new InvalidOperationException("Could not solve for the arc center.");

        double invDet = 1.0 / detM;

        double detX = M[0, 0] * (M[1, 1] * V[2] - M[1, 2] * V[1]) -
                      M[0, 1] * (M[1, 0] * V[2] - M[1, 2] * V[0]) +
                      M[0, 2] * (M[1, 0] * V[1] - M[1, 1] * V[0]);

        double detY = M[0, 0] * (V[1] * M[2, 2] - V[2] * M[1, 2]) -
                      V[0] * (M[1, 0] * M[2, 2] - M[1, 2] * M[2, 0]) +
                      M[0, 2] * (M[1, 0] * V[2] - M[1, 2] * V[0]);

        double detZ = M[0, 0] * (M[1, 1] * V[2] - M[1, 2] * V[1]) -
                      M[0, 1] * (M[1, 0] * V[2] - M[1, 2] * V[0]) +
                      V[0] * (M[1, 0] * M[2, 1] - M[1, 1] * M[2, 0]);

        // Ma trận nghịch đảo nhân với vector V để tìm nghiệm
        // Đây là cách viết khác của quy tắc Cramer
        double cx = ((M[1, 1] * M[2, 2] - M[1, 2] * M[2, 1]) * V[0] +
                      (M[0, 2] * M[2, 1] - M[0, 1] * M[2, 2]) * V[1] +
                      (M[0, 1] * M[1, 2] - M[0, 2] * M[1, 1]) * V[2]) * invDet;

        double cy = ((M[1, 2] * M[2, 0] - M[1, 0] * M[2, 2]) * V[0] +
                      (M[0, 0] * M[2, 2] - M[0, 2] * M[2, 0]) * V[1] +
                      (M[0, 2] * M[1, 0] - M[0, 0] * M[1, 2]) * V[2]) * invDet;

        double cz = ((M[1, 0] * M[2, 1] - M[1, 1] * M[2, 0]) * V[0] +
                      (M[0, 1] * M[2, 0] - M[0, 0] * M[2, 1]) * V[1] +
                      (M[0, 0] * M[1, 1] - M[0, 1] * M[1, 0]) * V[2]) * invDet;

        m_centerPoint = new Point3d(cx, cy, cz);
        m_radius = m_centerPoint.DistanceTo(A);
        m_plane = new GeoPlane(m_centerPoint, m_normal);

        Vector3d vStart = A - m_centerPoint;
        Vector3d vMid = B - m_centerPoint;
        Vector3d vEnd = C - m_centerPoint;

        double angle1 = vStart.GetAngleTo(vMid);
        if (vStart.CrossProduct(vMid).DotProduct(m_normal) < 0) angle1 = -angle1;

        double angle2 = vMid.GetAngleTo(vEnd);
        if (vMid.CrossProduct(vEnd).DotProduct(m_normal) < 0) angle2 = -angle2;

        m_angle = angle1 + angle2;
        if (m_angle < 0) m_angle += 2 * Math.PI;

        m_startAngle = 0.0;
        m_endAngle = m_angle;
        m_length = m_angle * m_radius;
    }

    /// <summary>
    /// Constructs a GeoArc from an AutoCAD Arc entity using its start, mid (at half length), and end points.
    /// </summary>
    /// <param name="arc">AutoCAD Arc entity</param>
    /// <exception cref="ArgumentNullException">Thrown if arc is null</exception>
    public GeoArc(Arc arc)
        : this(
            arc != null ? arc.StartPoint : throw new ArgumentNullException(nameof(arc)),
            arc.GetPointAtDist(arc.Length / 2.0),
            arc.EndPoint)
    {
    }

    #endregion

    #region Override

    /// <summary>
    /// Returns the local coordinate system at parameter t along the arc,
    /// where:
    /// - Y-axis is the tangent direction along the arc (from start to end),
    /// - Z-axis is the normal of the arc's plane,
    /// - X-axis is Y × Z (to the right),
    /// and Z is recomputed as X × Y to ensure orthonormality.
    /// </summary>
    public override GeoCoordinateSystem CoordinateSystemAtParameter(double t)
    {
        Point3d origin = PointAtParameter(t);
        Vector3d y = TangentAtParameter(t);             // Along the arc
        Vector3d z = m_normal.GetNormal();              // Plane normal

        Vector3d x = y.CrossProduct(z).GetNormal();     // Right of movement
        z = x.CrossProduct(y).GetNormal();              // Re-normalize Z

        return new GeoCoordinateSystem(origin, x, y, z);
    }

    /// <summary>
    /// Returns the local coordinate system at parameter t along the arc,
    /// with Z-axis fixed as global vertical.
    /// - Y-axis is the tangent direction along the arc,
    /// - X-axis is Y × Z (to the right),
    /// and Z is recomputed as X × Y to ensure orthonormality.
    /// </summary>
    public override GeoCoordinateSystem CoordinateSystemAtParameterFixZ(double t)
    {
        Point3d origin = PointAtParameter(t);
        Vector3d y = TangentAtParameter(t);             // Arc direction
        Vector3d z = Vector3d.ZAxis;                    // Global vertical

        Vector3d x = y.CrossProduct(z).GetNormal();

        if (x.Length < Constants.DefaultTolerance)
            x = Vector3d.XAxis;

        z = x.CrossProduct(y).GetNormal();              // Re-adjust Z

        return new GeoCoordinateSystem(origin, x, y, z);
    }

    /// <summary>
    /// Returns the 3D point corresponding to parameter t (0 to 1) on the arc.
    /// </summary>
    public override Point3d PointAtParameter(double t)
    {
        double angle = t * m_angle;

        Vector3d startVec = (m_startpoint - m_centerPoint).GetNormal();
        Vector3d orthoVec = m_normal.CrossProduct(startVec).GetNormal();

        Vector3d offset = Math.Cos(angle) * startVec + Math.Sin(angle) * orthoVec;

        return m_centerPoint + m_radius * offset;
    }

    /// <summary>
    /// Returns the local coordinate system at a given arc length from the start point.
    /// </summary>
    public override GeoCoordinateSystem CoordinateSystemAtSegmentLength(double t)
    {
        double param = t / m_length;
        return CoordinateSystemAtParameter(param);
    }

    /// <summary>
    /// Returns the normal vector (pointing inward) at a given parameter t along the arc.
    /// </summary>
    public override Vector3d NormalAtParameter(double t)
    {
        double angle = t * m_angle;

        Vector3d X = (m_startpoint - m_centerPoint).GetNormal();
        Vector3d Y = m_normal.CrossProduct(X).GetNormal();

        Vector3d normal = -Math.Cos(angle) * X - Math.Sin(angle) * Y;

        return normal.GetNormal();
    }

    /// <summary>
    /// Returns the parameter t (0 to 1) corresponding to the projection of a point onto the arc.
    /// </summary>
    public override double ParameterAtPoint(Point3d point)
    {
        Vector3d from = (point - m_centerPoint).GetNormal();
        Vector3d startVec = (m_startpoint - m_centerPoint).GetNormal();
        Vector3d orthoVec = m_normal.CrossProduct(startVec).GetNormal();

        double cos = from.DotProduct(startVec);
        double sin = from.DotProduct(orthoVec);

        double angle = Math.Atan2(sin, cos);
        if (angle < 0) angle += 2 * Math.PI;

        double t = angle / m_angle;
        return Math.Max(0, Math.Min(1, t));
    }

    /// <summary>
    /// Returns a perpendicular plane at parameter t on the arc.
    /// </summary>
    public override GeoPlane PlaneAtParameter(double t)
    {
        return new GeoPlane(PointAtParameter(t), NormalAtParameter(t));
    }

    /// <summary>
    /// Projects the arc onto a target plane.
    /// </summary>
    public override GeoCurves PullOntoPlane(GeoPlane plane)
    {
        Vector3d n = plane.Normal;

        Point3d project(Point3d pt)
        {
            double d = (pt - plane.Origin).DotProduct(n);
            return pt - d * n;
        }

        return new GeoArc(
            project(m_startpoint),
            project(m_midPoint),
            project(m_endpoint)
        );
    }

    /// <summary>
    /// Reverses the arc direction (start ↔ end).
    /// </summary>
    public override void Reverse()
    {
        // Đổi start và end point
        var temp = m_startpoint;
        m_startpoint = m_endpoint;
        m_endpoint = temp;

        // Tính lại midpoint theo hướng mới
        m_midPoint = PointAtParameter(0.5);

        // Dựng lại toàn bộ hình học bằng constructor tạm thời
        var reversed = new GeoArc(m_startpoint, m_midPoint, m_endpoint);

        // Cập nhật lại toàn bộ thuộc tính hình học
        m_centerPoint = reversed.CenterPoint;
        m_normal = reversed.Normal;
        m_plane = reversed.Plane;
        m_radius = reversed.Radius;
        m_angle = reversed.Angle;
        m_length = reversed.Length;
        m_startAngle = reversed.StartAngle;
        m_endAngle = reversed.EndAngle;
        m_isClosed = reversed.IsClosed;
    }

    /// <summary>
    /// Splits the arc at the specified list of normalized parameters (0 to 1).
    /// </summary>
    public override List<GeoCurves> SplitByParameter(List<double> parameters)
    {
        var result = new List<GeoCurves>();

        if (parameters == null || parameters.Count == 0)
        {
            result.Add(this);
            return result;
        }

        // Clamp + sort t values
        var tValues = parameters
            .Select(t => Math.Max(0, Math.Min(1, t)))
            .Concat(new[] { 0.0, 1.0 }) // Bắt buộc có đầu cuối
            .Distinct()
            .OrderBy(t => t)
            .ToList();

        // Lấy các điểm theo t
        var points = tValues.Select(PointAtParameter).ToList();

        // Duyệt từng cặp điểm liên tiếp → tạo GeoArc
        for (int i = 0; i < points.Count - 1; i++)
        {
            var pt1 = points[i];
            var pt2 = points[i + 1];
            var mid = PointAtParameter((tValues[i] + tValues[i + 1]) / 2.0);

            try
            {
                result.Add(new GeoArc(pt1, mid, pt2));
            }
            catch
            {
                // Bỏ qua đoạn nếu lỗi (trùng điểm, colinear, ...)
            }
        }

        return result;
    }

    /// <summary>
    /// Splits the arc at the projections of specified points.
    /// </summary>
    public override List<GeoCurves> SplitByPoints(List<Point3d> points)
    {
        if (points == null || points.Count == 0)
            return new List<GeoCurves> { this };

        var parameters = points
            .Select(p => ParameterAtPoint(p))
            .Distinct()
            .OrderBy(p => p)
            .ToList();

        return SplitByParameter(parameters);
    }

    /// <summary>
    /// Returns the shortest distance from a point to the arc.
    /// </summary>
    public override double DistanceTo(Point3d point)
    {
        Vector3d from = (StartPoint - CenterPoint).GetNormal();
        Vector3d ortho = Normal.CrossProduct(from).GetNormal();
        Vector3d to = point - CenterPoint;

        double x = to.DotProduct(from);
        double y = to.DotProduct(ortho);

        double angle = Math.Atan2(y, x);
        if (angle < 0) angle += 2 * Math.PI;

        if (angle > Angle) angle = Angle;

        double t = angle / Angle;
        Point3d closest = PointAtParameter(t);
        return point.DistanceTo(closest);
    }

    /// <summary>
    /// Computes intersection between this arc and another curve (line, arc, or polycurve).
    /// </summary>
    public override IntersectionResult IntersectWith(GeoCurves other)
    {
        if (other is GeoLine line)
            return line.IntersectWith(this); // delegate to GeoLine's logic
        else if (other is GeoArc arc)
            return IntersectWithArc(arc);
        else if (other is GeoPolycurve poly)
            return IntersectWithPolycurve(poly);

        throw new NotSupportedException($"GeoArc does not support intersection with {other.GetType().Name}");
    }

    public override Entity[] Test()
    {
        try
        {
            Point3d realCenter = this.CenterPoint;
            Vector3d realNormal = this.Normal;
            double realRadius = this.Radius;

            Vector3d ocsX;
            double tolerance = 1.0 / 64.0;

            if (Math.Abs(realNormal.X) < tolerance && Math.Abs(realNormal.Y) < tolerance)
            {
                ocsX = Vector3d.YAxis.CrossProduct(realNormal).GetNormal();
            }
            else
            {
                ocsX = Vector3d.ZAxis.CrossProduct(realNormal).GetNormal();
            }

            Vector3d ocsY = realNormal.CrossProduct(ocsX).GetNormal();

            Vector3d startVector = this.StartPoint - realCenter;

            double startAngleInOCS = Math.Atan2(startVector.DotProduct(ocsY), startVector.DotProduct(ocsX));

            double endAngleInOCS = startAngleInOCS + this.Angle;

            var arc = new Arc(
                realCenter,
                realNormal,
                realRadius,
                startAngleInOCS,
                endAngleInOCS
            )
            {
                ColorIndex = 4
            };

            return new Entity[] { arc };
        }
        catch (System.Exception ex)
        {
            System.Diagnostics.Debug.WriteLine($"Error creating Arc for Test(): {ex.Message}");
            return new Entity[0];
        }
    }

    #endregion

    #region Private

    /// <summary>
    /// Computes intersection between two arcs, assuming they lie on the same plane.
    /// </summary>
    /// <param name="other">The other arc to intersect with.</param>
    /// <returns>Intersection result with 0, 1 or 2 points.</returns>
    public IntersectionResult IntersectWithArc(GeoArc other)
    {
        Editor ed = Application.DocumentManager.MdiActiveDocument.Editor;

        // 1. Kiểm tra đồng phẳng (normal cùng hướng và tâm cùng mặt phẳng)
        double dotNorm = this.Normal.DotProduct(other.Normal);
        double planeOffset = (this.CenterPoint - other.CenterPoint).DotProduct(this.Normal);

        if (Math.Abs(planeOffset) > 1e-6 || Math.Abs(dotNorm - 1.0) > 1e-6)
        {
            ed.WriteMessage("\n❌ Hai cung không đồng phẳng hoặc không cùng mặt.");
            return IntersectionResult.None();
        }

        ed.WriteMessage("\n✅ Hai cung đồng phẳng.");

        // 2. Đưa về hệ trục 2D của cung this
        Vector3d xAxis = (this.StartPoint - this.CenterPoint).GetNormal();
        Vector3d yAxis = this.Normal.CrossProduct(xAxis).GetNormal();

        Point2d To2D(Point3d p)
        {
            Vector3d v = p - this.CenterPoint;
            return new Point2d(v.DotProduct(xAxis), v.DotProduct(yAxis));
        }

        Point3d To3D(Point2d pt)
        {
            return this.CenterPoint + xAxis * pt.X + yAxis * pt.Y;
        }

        Point2d c1 = new Point2d(0, 0); // Center cung this (sau khi quy về gốc)
        Point2d c2 = To2D(other.CenterPoint);

        double r1 = this.Radius;
        double r2 = other.Radius;

        Vector2d d = c2 - c1;
        double D = d.Length;

        if (D > r1 + r2 || D < Math.Abs(r1 - r2) || D < 1e-6)
        {
            ed.WriteMessage("\n❌ Hai vòng tròn không giao nhau.");
            return IntersectionResult.None();
        }

        // 3. Giao hình học 2 đường tròn
        double a = (r1 * r1 - r2 * r2 + D * D) / (2 * D);
        double h = Math.Sqrt(r1 * r1 - a * a);
        Point2d p0 = c1 + d.GetNormal() * a;

        Vector2d offset = new Vector2d(-d.Y, d.X).GetNormal() * h;
        Point2d ptA = p0 + offset;
        Point2d ptB = p0 - offset;

        List<Point3d> intersections = new List<Point3d>();

        foreach (Point2d pt2D in new Point2d[] { ptA, ptB })
        {
            Point3d pt3D = To3D(pt2D);

            if (!IsPointOnArc(this, pt3D))
            {
                ed.WriteMessage("\n⚠️ Giao điểm không nằm trên cung this.");
                continue;
            }

            if (!IsPointOnArc(other, pt3D))
            {
                ed.WriteMessage("\n⚠️ Giao điểm không nằm trên cung other.");
                continue;
            }

            ed.WriteMessage("\n✅ Giao điểm hợp lệ: " + pt3D.ToString());
            intersections.Add(pt3D);
        }

        if (intersections.Count == 0)
            return IntersectionResult.None();

        return IntersectionResult.Intersect(intersections);
    }

    /// <summary>
    /// Computes intersection between this arc and a polycurve (by testing each segment).
    /// </summary>
    /// <param name="poly">Target polycurve.</param>
    private IntersectionResult IntersectWithPolycurve(GeoPolycurve poly)
    {
        if (poly == null || poly.Segments == null || poly.Segments.Count == 0)
            return IntersectionResult.None();

        List<Point3d> points = new List<Point3d>();
        List<GeoCurves> overlaps = new List<GeoCurves>();

        foreach (GeoCurves segment in poly.Segments)
        {
            IntersectionResult result = this.IntersectWith(segment);

            if (result == null || result.Type == IntersectionResult.IntersectType.None)
                continue;

            if (result.Type == IntersectionResult.IntersectType.Intersect && result.Points != null)
            {
                foreach (Point3d pt in result.Points)
                {
                    if (!points.Any(p => p.DistanceTo(pt) < Constants.DefaultTolerance))
                        points.Add(pt);
                }
            }

            if ((result.Type == IntersectionResult.IntersectType.Overlap ||
                 result.Type == IntersectionResult.IntersectType.SameLine)
                && result.OverlapSegments != null)
            {
                overlaps.AddRange(result.OverlapSegments);
            }
        }

        if (overlaps.Count > 0)
            return IntersectionResult.Overlap(overlaps);

        if (points.Count > 0)
            return IntersectionResult.Intersect(points);

        return IntersectionResult.None();
    }

    /// <summary>
    /// Checks whether a point lies within the arc's sweep (from StartPoint to EndPoint).
    /// </summary>
    /// <param name="arc">The arc to check against.</param>
    /// <param name="pt">The point to check.</param>
    private bool IsPointOnArc(GeoArc arc, Point3d pt)
    {
        var v = (pt - arc.CenterPoint).GetNormal();
        var vStart = (arc.StartPoint - arc.CenterPoint).GetNormal();

        double angle = vStart.GetAngleTo(v);
        if (vStart.CrossProduct(v).DotProduct(arc.Normal) < 0)
            angle = 2 * Math.PI - angle;

        return angle >= -Constants.DefaultTolerance && angle <= arc.Angle + Constants.DefaultTolerance;
    }

    /// <summary>
    /// Returns the signed angle between two vectors on a given plane.
    /// </summary>
    private double Vector3dAngleOnPlane(Vector3d from, Vector3d to, Vector3d planeNormal)
    {
        double angle = from.GetAngleTo(to);
        Vector3d cross = from.CrossProduct(to);
        return cross.DotProduct(planeNormal) < 0 ? 2 * Math.PI - angle : angle;
    }

    #endregion

    #region Public Method
    /// <summary>
    /// Returns a formatted string containing geometric information of the GeoArc.
    /// </summary>
    public string GetInfoString()
    {
        var sb = new StringBuilder();

        sb.AppendLine("===== GeoArc Geometry Info =====");

        sb.AppendLine($"Start X\t{StartPoint.X:0.####}");
        sb.AppendLine($"Start Y\t{StartPoint.Y:0.####}");
        sb.AppendLine($"Start Z\t{StartPoint.Z:0.####}");

        sb.AppendLine($"Center X\t{CenterPoint.X:0.####}");
        sb.AppendLine($"Center Y\t{CenterPoint.Y:0.####}");
        sb.AppendLine($"Center Z\t{CenterPoint.Z:0.####}");

        sb.AppendLine($"End X\t{EndPoint.X:0.####}");
        sb.AppendLine($"End Y\t{EndPoint.Y:0.####}");
        sb.AppendLine($"End Z\t{EndPoint.Z:0.####}");

        sb.AppendLine($"Radius\t{Radius:0.####}");

        var xAxis = (StartPoint - CenterPoint).GetNormal();
        var vEnd = (EndPoint - CenterPoint).GetNormal();

        double startAngle = 0;
        double endAngle = xAxis.GetAngleTo(vEnd, Normal);
        double totalAngle = Angle;
        double arcLength = Radius * Angle;
        double area = 0.5 * Radius * Radius * Angle;

        sb.AppendLine($"Start angle\t{startAngle * 180 / Math.PI:0.####}");
        sb.AppendLine($"End angle\t{endAngle * 180 / Math.PI:0.####}");
        sb.AppendLine($"Total angle\t{totalAngle * 180 / Math.PI:0.####}");
        sb.AppendLine($"Arc length\t{arcLength:0.####}");
        sb.AppendLine($"Area\t{area:0.####}");

        sb.AppendLine($"Normal X\t{Normal.X:0.####}");
        sb.AppendLine($"Normal Y\t{Normal.Y:0.####}");
        sb.AppendLine($"Normal Z\t{Normal.Z:0.####}");

        sb.AppendLine("===============================");

        return sb.ToString();
    }

    /// <summary>
    /// Determines whether the given 3D point lies on the arc (not full circle).
    /// </summary>
    /// <param name="point">3D point to check</param>
    /// <returns>True if point lies on the arc, false otherwise</returns>
    public bool IsPointOnArc(Point3d point)
    {
        // 1. Kiểm tra điểm có nằm trên đường tròn vô hạn không
        if (Math.Abs(point.DistanceTo(this.CenterPoint) - this.Radius) > Constants.DefaultTolerance)
        {
            return false;
        }

        // 2. Kiểm tra góc của điểm có nằm trong khoảng góc của cung không
        Vector3d vecToPoint = point - this.CenterPoint;
        Vector3d xAxis = (this.StartPoint - this.CenterPoint).GetNormal();

        // Tính góc của điểm so với trục X của cung
        double angle = Math.Atan2(vecToPoint.DotProduct(this.Normal.CrossProduct(xAxis)), vecToPoint.DotProduct(xAxis));

        // Chuẩn hóa góc về khoảng [0, 2*PI)
        if (angle < 0) angle += 2 * Math.PI;

        double start = this.StartAngle;
        double end = this.EndAngle;

        // Xử lý trường hợp cung đi ngược chiều kim đồng hồ qua góc 0
        if (end < start)
        {
            return (angle >= start || angle <= end);
        }

        return (angle >= start && angle <= end);
    }

    /// <summary>
    /// Returns the tangent vector at a parameter t (from 0 to 1) along the arc.
    /// The tangent is the derivative of the arc position at angle θ = t × sweep angle.
    /// </summary>
    /// <param name="t">Normalized parameter along the arc (0 = start, 1 = end).</param>
    /// <returns>Normalized tangent vector pointing along the arc direction.</returns>
    public Vector3d TangentAtParameter(double t)
    {
        double angle = t * m_angle;

        Vector3d startVec = (m_startpoint - m_centerPoint).GetNormal();                // X-axis
        Vector3d orthoVec = m_normal.CrossProduct(startVec).GetNormal();              // Y-axis (right-handed)

        // Tangent = d/dθ (position) = -sinθ·X + cosθ·Y
        Vector3d tangent = -Math.Sin(angle) * startVec + Math.Cos(angle) * orthoVec;

        return tangent.GetNormal();
    }

    /// <summary>
    /// Tính tham số t (0 ≤ t ≤ 1) của một điểm bất kỳ so với GeoArc,
    /// bằng cách chiếu toàn bộ cung tròn và điểm kiểm tra xuống mặt phẳng OXY.
    /// </summary>
    /// <param name="point">Điểm cần tính t</param>
    /// <returns>Giá trị t đã clamp</returns>
    /// <summary>
    /// Tính tham số t (0 ≤ t ≤ 1) của một điểm bất kỳ so với GeoArc,
    /// bằng cách chiếu toàn bộ cung tròn và điểm kiểm tra xuống mặt phẳng OXY.
    /// </summary>
    /// <param name="point">Điểm cần tính t</param>
    /// <returns>Giá trị t đã clamp</returns>
    public override double ParameterAtPoint2D(Point3d point)
    {
        // 1. Chiếu điểm tâm + start + normal xuống mặt phẳng OXY
        Point3d center = new Point3d(CenterPoint.X, CenterPoint.Y, 0);
        Point3d start = new Point3d(StartPoint.X, StartPoint.Y, 0);
        Point3d test = new Point3d(point.X, point.Y, 0);

        // 2. Vector bán kính bắt đầu
        Vector3d from = (start - center).GetNormal();

        // 3. Vector vuông góc (theo chiều quay, XOY ⇒ Z lên trên)
        Vector3d z = Vector3d.ZAxis; // ép dùng OXY luôn
        Vector3d ortho = z.CrossProduct(from).GetNormal();

        // 4. Vector từ tâm đến điểm test
        Vector3d to = (test - center).GetNormal();

        // 5. Tính cos/sin → Atan2
        double cos = to.DotProduct(from);
        double sin = to.DotProduct(ortho);

        double angle = Math.Atan2(sin, cos);
        if (angle < 0) angle += 2 * Math.PI;

        // 6. Clamp theo góc quét
        if (angle > Angle) angle = Angle;

        // 7. Nội suy t
        double t = angle / Angle;
        return Math.Max(0, Math.Min(1, t));
    }

    #endregion
}

public sealed class GeoPolycurve: GeoCurves
{
    #region Fields

    private Point3d _startPoint;         // Start point of the polycurve
    private Point3d _endPoint;           // End point of the polycurve
    private bool m_isClosed;             // Whether the polycurve forms a closed loop
    private double _length;              // Total length of the polycurve

    private List<GeoCurves> segments;    // List of curve segments (lines, arcs, etc.)
    private List<Point3d> vertices;      // List of key vertices (usually endpoints of segments)

    #endregion

    #region Properties

    public override Point3d StartPoint { get => _startPoint; protected set => _startPoint = value; }  // First point of the polycurve
    public override Point3d EndPoint { get => _endPoint; protected set => _endPoint = value; }    // Last point of the polycurve
    public override bool IsClosed { get => m_isClosed; protected set => m_isClosed = value; }   // Whether the polycurve is closed
    public override double Length { get => _length; protected set => _length = value; }      // Total length of all segments
    public List<GeoCurves> Segments { get => segments; set => segments = value; }               // All curve segments
    public List<Point3d> Vertices { get => vertices; set => vertices = value; }               // Vertex points between segments

    #endregion

    #region Private

    /// <summary>
    /// Computes the index of the segment and local parameter (0..1) corresponding to a normalized t (0..1) on the full polycurve.
    /// </summary>
    /// <param name="t">Normalized parameter along the full polycurve (0 to 1).</param>
    /// <returns>A tuple: (segment index, local parameter on that segment).</returns>
    /// <exception cref="InvalidOperationException">Thrown when no segments are defined.</exception>
    private (int index, double localT) GetSegmentAndLocalT(double t)
    {
        if (Segments == null || Segments.Count == 0)
            throw new InvalidOperationException("GeoPolycurve does not contain any segments.");

        double totalLength = Length;
        double targetLength = t * totalLength;
        double accumulated = 0;

        for (int i = 0; i < Segments.Count; i++)
        {
            double segLength = Segments[i].Length;

            if (accumulated + segLength >= targetLength - Constants.DefaultTolerance)
            {
                double localT = (targetLength - accumulated) / segLength;
                return (i, localT);
            }

            accumulated += segLength;
        }

        return (Segments.Count - 1, 1.0); // fallback to last segment
    }

    /// <summary>
    /// Computes the index of the segment and local parameter (0..1) given an absolute length from the start of the polycurve.
    /// </summary>
    /// <param name="absLength">Absolute length from the start of the polycurve (in meters).</param>
    /// <returns>A tuple: (segment index, local parameter on that segment).</returns>
    /// <exception cref="InvalidOperationException">Thrown when no segments are defined.</exception>
    private (int index, double localT) GetSegmentAndLocalTByLength(double absLength)
    {
        if (Segments == null || Segments.Count == 0)
            throw new InvalidOperationException("GeoPolycurve does not contain any segments.");

        double accumulated = 0;

        for (int i = 0; i < Segments.Count; i++)
        {
            double segLength = Segments[i].Length;

            if (accumulated + segLength >= absLength - Constants.DefaultTolerance)
            {
                double localT = (absLength - accumulated) / segLength;
                return (i, localT);
            }

            accumulated += segLength;
        }

        return (Segments.Count - 1, 1.0); // fallback to last segment
    }

    /// <summary>
    /// Checks if the angle formed by three consecutive 2D points (a → b → c) is convex.
    /// </summaryprivate>
    /// <param name="a">First point</param>
    /// <param name="b">Middle point (vertex)</param>
    /// <param name="c">Third point</param>
    /// <returns>True if the angle is convex (counter-clockwise)</returns>
    private bool IsConvex(Point2d a, Point2d b, Point2d c)
    {
        var ab = b - a;
        var bc = c - b;
        return ab.X * bc.Y - ab.Y * bc.X > 0;
    }

    /// <summary>
    /// Checks whether a point lies inside the triangle defined by three 2D points.
    /// </summary>
    /// <param name="pt">The point to check</param>
    /// <param name="a">Triangle vertex A</param>
    /// <param name="b">Triangle vertex B</param>
    /// <param name="c">Triangle vertex C</param>
    /// <returns>True if the point is inside the triangle (including on edges)</returns>
    private bool IsPointInTriangle(Point2d pt, Point2d a, Point2d b, Point2d c)
    {
        var v0 = c - a;
        var v1 = b - a;
        var v2 = pt - a;

        double dot00 = v0.DotProduct(v0);
        double dot01 = v0.DotProduct(v1);
        double dot02 = v0.DotProduct(v2);
        double dot11 = v1.DotProduct(v1);
        double dot12 = v1.DotProduct(v2);

        double denom = dot00 * dot11 - dot01 * dot01;
        if (denom == 0) return false;

        double u = (dot11 * dot02 - dot01 * dot12) / denom;
        double v = (dot00 * dot12 - dot01 * dot02) / denom;

        return u >= 0 && v >= 0 && (u + v) <= 1;
    }

    /// <summary>
    /// Computes the signed area of a closed polygon defined by a list of 2D points.
    /// </summary>
    /// <param name="pts">List of points defining the polygon (must be closed or treated as loop)</param>
    /// <returns>Signed area (positive if counter-clockwise, negative if clockwise)</returns>
    private double GetSignedArea(List<Point2d> pts)
    {
        double area = 0;
        for (int i = 0; i < pts.Count; i++)
        {
            var p1 = pts[i];
            var p2 = pts[(i + 1) % pts.Count];
            area += (p1.X * p2.Y - p2.X * p1.Y);
        }
        return 0.5 * area;
    }
    #endregion

    #region Override

    /// <summary>
    /// Returns the coordinate system at a given normalized parameter t along the full polycurve.
    /// </summary>
    /// <summary>
    /// Returns the local coordinate system at parameter t along the polycurve.
    /// This delegates the computation to the corresponding sub-segment (GeoLine or GeoArc),
    /// preserving the local behavior (e.g., using arc normal for Z).
    /// </summary>
    public override GeoCoordinateSystem CoordinateSystemAtParameter(double t)
    {
        var (index, localT) = GetSegmentAndLocalT(t);
        return Segments[index].CoordinateSystemAtParameter(localT);
    }

    /// <summary>
    /// Returns the local coordinate system at parameter t along the polycurve,
    /// with Z-axis fixed as global vertical (Vector3d.ZAxis).
    /// The computation is delegated to the corresponding segment.
    /// </summary>
    public override GeoCoordinateSystem CoordinateSystemAtParameterFixZ(double t)
    {
        var (index, localT) = GetSegmentAndLocalT(t);
        return Segments[index].CoordinateSystemAtParameterFixZ(localT);
    }

    /// <summary>
    /// Returns the coordinate system at a distance t from the start of the polycurve.
    /// </summary>
    public override GeoCoordinateSystem CoordinateSystemAtSegmentLength(double t)
    {
        var (index, localT) = GetSegmentAndLocalTByLength(t);
        return Segments[index].CoordinateSystemAtParameter(localT);
    }

    /// <summary>
    /// Returns the normal vector at a given parameter t along the polycurve.
    /// </summary>
    public override Vector3d NormalAtParameter(double t)
    {
        if (Segments == null || Segments.Count == 0)
            throw new InvalidOperationException("GeoPolycurve does not contain any segments.");

        var (index, localT) = GetSegmentAndLocalT(t);
        return Segments[index].NormalAtParameter(localT);
    }

    /// <summary>
    /// Finds the parameter t (0 to 1) where the polycurve is closest to a given point.
    /// </summary>
    public override double ParameterAtPoint(Point3d point)
    {
        if (Segments == null || Segments.Count == 0)
            throw new InvalidOperationException("GeoPolycurve does not contain any segments.");

        double accumulatedLength = 0;
        double totalLength = Length;
        double bestT = 0;
        double bestDistance = double.MaxValue;

        foreach (var segment in Segments)
        {
            double localT = segment.ParameterAtPoint(point);
            Point3d closestPt = segment.PointAtParameter(localT);
            double distance = closestPt.DistanceTo(point);

            if (distance < bestDistance)
            {
                bestDistance = distance;
                double globalLengthAtPoint = accumulatedLength + localT * segment.Length;
                bestT = globalLengthAtPoint / totalLength;
            }

            accumulatedLength += segment.Length;
        }

        return bestT;
    }

    /// <summary>
    /// Returns the perpendicular plane at a given parameter t along the polycurve.
    /// </summary>
    public override GeoPlane PlaneAtParameter(double t)
    {
        if (Segments == null || Segments.Count == 0)
            throw new InvalidOperationException("GeoPolycurve does not contain any segments.");

        var (index, localT) = GetSegmentAndLocalT(t);
        return Segments[index].PlaneAtParameter(localT);
    }

    /// <summary>
    /// Computes the minimum distance from the polycurve to a point.
    /// </summary>
    public override double DistanceTo(Point3d point)
    {
        return Segments.Min(seg => seg.DistanceTo(point));
    }

    /// <summary>
    /// Returns the 3D point at a normalized parameter t on the polycurve.
    /// </summary>
    public override Point3d PointAtParameter(double t)
    {
        if (Segments == null || Segments.Count == 0)
            throw new InvalidOperationException("GeoPolycurve does not contain any segments.");

        var (index, localT) = GetSegmentAndLocalT(t);
        return Segments[index].PointAtParameter(localT);
    }

    /// <summary>
    /// Projects the entire polycurve onto a target plane.
    /// </summary>
    public override GeoCurves PullOntoPlane(GeoPlane plane)
    {
        var projectedSegments = Segments.Select(s => s.PullOntoPlane(plane)).ToList();
        return new GeoPolycurve(projectedSegments);
    }

    /// <summary>
    /// Reverses the direction of the polycurve and all its segments.
    /// </summary>
    public static List<GeoPolycurve> PullMultipleOntoPlane(List<GeoCurves> segments, GeoPlane plane)
    {
        var projected = segments.Select(s => s.PullOntoPlane(plane)).ToList();
        return GeoPolycurve.ByGroupCurves(projected);
    }

    /// <summary>
    /// Splits the polycurve into sub-curves at given normalized parameters (0 to 1).
    /// </summary>
    public override List<GeoCurves> SplitByParameter(List<double> parameters)
    {
        if (parameters == null || parameters.Count == 0)
            return new List<GeoCurves> { this };

        var sortedParams = parameters
            .Select(p => Math.Max(0, Math.Min(1, p)))
            .Distinct()
            .OrderBy(p => p)
            .ToList();

        var result = new List<GeoCurves>();
        double prevT = 0;

        foreach (var t in sortedParams)
        {
            if (Math.Abs(t - prevT) < Constants.DefaultTolerance) continue;

            var (startIndex, localT1) = GetSegmentAndLocalT(prevT);
            var (endIndex, localT2) = GetSegmentAndLocalT(t);

            if (startIndex == endIndex)
            {
                var piece = Segments[startIndex].SplitByParameter(new List<double> { localT1, localT2 });
                result.Add(piece[1]);
            }
            else
            {
                var startSplit = Segments[startIndex].SplitByParameter(new List<double> { localT1 });
                result.Add(startSplit[1]);

                for (int i = startIndex + 1; i < endIndex; i++)
                    result.Add(Segments[i]);

                var endSplit = Segments[endIndex].SplitByParameter(new List<double> { localT2 });
                result.Add(endSplit[0]);
            }

            prevT = t;
        }

        if (prevT < 1.0 - Constants.DefaultTolerance)
        {
            var (startIndex, localT1) = GetSegmentAndLocalT(prevT);
            var endSplit = Segments[startIndex].SplitByParameter(new List<double> { localT1 });
            result.Add(endSplit[1]);
        }

        return result;
    }

    /// <summary>
    /// Splits the polycurve at the closest projection points to the input point list.
    /// </summary>
    public override List<GeoCurves> SplitByPoints(List<Point3d> points)
    {
        if (points == null || points.Count == 0)
            return new List<GeoCurves> { this };

        var parameters = points
            .Select(p => ParameterAtPoint(p))
            .Distinct()
            .OrderBy(p => p)
            .ToList();

        return SplitByParameter(parameters);
    }

    /// <summary>
    /// Computes the intersection between the polycurve and another curve.
    /// </summary>
    public override IntersectionResult IntersectWith(GeoCurves other)
    {
        if (Segments == null || Segments.Count == 0 || other == null)
            return IntersectionResult.None();

        List<Point3d> allPoints = new List<Point3d>();
        List<GeoCurves> allOverlaps = new List<GeoCurves>();

        foreach (GeoCurves segment in Segments)
        {
            var result = segment.IntersectWith(other);

            if (result == null || result.Type == IntersectionResult.IntersectType.None)
                continue;

            if (result.Type == IntersectionResult.IntersectType.Intersect && result.Points != null)
            {
                foreach (var pt in result.Points)
                {
                    if (!allPoints.Any(p => p.DistanceTo(pt) < Constants.DefaultTolerance))
                        allPoints.Add(pt);
                }
            }

            if ((result.Type == IntersectionResult.IntersectType.Overlap ||
                 result.Type == IntersectionResult.IntersectType.SameLine)
                && result.OverlapSegments != null)
            {
                allOverlaps.AddRange(result.OverlapSegments);
            }
        }

        if (allOverlaps.Count > 0)
            return IntersectionResult.Overlap(allOverlaps);

        if (allPoints.Count > 0)
            return IntersectionResult.Intersect(allPoints);

        return IntersectionResult.None();
    }

    public override void Reverse()
    {
        // Đảo hướng từng segment + đảo thứ tự toàn bộ
        Segments = Segments.Select(s => { s.Reverse(); return s; }).Reverse().ToList();

        // Cập nhật lại Vertices
        Vertices = Segments.Select(s => s.StartPoint).ToList();
        Vertices.Add(Segments.Last().EndPoint);

        // Cập nhật lại start/end point
        _startPoint = Vertices.First();
        _endPoint = Vertices.Last();
    }

    public override Entity[] Test()
    {
        var pts = new Point3dCollection();
        foreach (var pt in Vertices)
            pts.Add(pt);

        var polyline = new Polyline3d(Poly3dType.SimplePoly, pts, false)
        {
            ColorIndex = 4
        };

        return new Entity[] { polyline };
    }

    /// <summary>
    /// Tính tham số t (0 ≤ t ≤ 1) của một điểm bất kỳ so với GeoPolycurve,
    /// bằng cách chiếu toàn bộ các đoạn xuống mặt phẳng OXY và tìm đoạn gần nhất.
    /// </summary>
    /// <param name="point">Điểm kiểm tra</param>
    /// <returns>Giá trị t toàn cục trong polycurve (chuẩn hóa từ 0 đến 1)</returns>
    public override double ParameterAtPoint2D(Point3d point)
    {
        if (Segments == null || Segments.Count == 0)
            throw new InvalidOperationException("GeoPolycurve không có đoạn nào.");

        Point3d test = new Point3d(point.X, point.Y, 0); // Chiếu xuống OXY

        double closestT = 0;
        double accumulated = 0;
        double bestDistance = double.MaxValue;

        foreach (var segment in Segments)
        {
            Point3d p0 = new Point3d(segment.StartPoint.X, segment.StartPoint.Y, 0);
            Point3d p1 = new Point3d(segment.EndPoint.X, segment.EndPoint.Y, 0);

            Vector3d lineVec = p1 - p0;
            Vector3d toTest = test - p0;

            double len2 = lineVec.LengthSqrd;
            if (len2 < Constants.DefaultTolerance)
            {
                accumulated += segment.Length;
                continue;
            }

            double tLocal = toTest.DotProduct(lineVec) / len2;
            tLocal = Math.Max(0, Math.Min(1, tLocal));

            Point3d projected = p0 + tLocal * lineVec;
            double distance = test.DistanceTo(projected);

            if (distance < bestDistance)
            {
                bestDistance = distance;

                double absLength = accumulated + tLocal * segment.Length;
                closestT = absLength / Length;
            }

            accumulated += segment.Length;
        }

        return closestT;
    }
    #endregion

    #region Constructor

    /// <summary>
    /// Initializes a new <see cref="GeoPolycurve"/> by connecting a list of continuous <see cref="GeoCurves"/> segments.
    /// </summary>
    /// <param name="curves">The list of curve segments to connect into a single polycurve.</param>
    /// <exception cref="ArgumentException">
    /// Thrown if:
    /// <list type="bullet">
    /// <item><description>The list is null or empty.</description></item>
    /// <item><description>The segments cannot be connected into a single continuous chain.</description></item>
    /// </list>
    /// </exception>
    /// <remarks>
    /// The segments will be automatically ordered and reversed (if necessary) to form a continuous polycurve.
    /// </remarks>
    public GeoPolycurve(List<GeoCurves> curves)
    {
        if (curves == null || curves.Count == 0)
            throw new ArgumentException("GeoPolycurve requires at least one segment.");

        var remaining = new List<GeoCurves>(curves);
        var points = new LinkedList<Point3d>();
        var segments = new LinkedList<GeoCurves>();

        var seed = remaining[0];
        remaining.RemoveAt(0);
        points.AddLast(seed.StartPoint);
        points.AddLast(seed.EndPoint);
        segments.AddLast(seed);

        bool extended;
        do
        {
            extended = false;

            for (int i = 0; i < remaining.Count; i++)
            {
                var c = remaining[i];
                if (points.Last.Value.IsEqualTo(c.StartPoint)) { segments.AddLast(c); points.AddLast(c.EndPoint); remaining.RemoveAt(i); extended = true; break; }
                if (points.Last.Value.IsEqualTo(c.EndPoint)) { c.Reverse(); segments.AddLast(c); points.AddLast(c.EndPoint); remaining.RemoveAt(i); extended = true; break; }
                if (points.First.Value.IsEqualTo(c.EndPoint)) { c.Reverse(); segments.AddFirst(c); points.AddFirst(c.StartPoint); remaining.RemoveAt(i); extended = true; break; }
                if (points.First.Value.IsEqualTo(c.StartPoint)) { segments.AddFirst(c); points.AddFirst(c.EndPoint); remaining.RemoveAt(i); extended = true; break; }
            }

        } while (extended);

        if (remaining.Count > 0)
            throw new ArgumentException("Không thể ghép toàn bộ thành một polycurve duy nhất.");

        Segments = segments.ToList();
        Vertices = points.ToList();
        _startPoint = Vertices.First();
        _endPoint = Vertices.Last();
        _length = Segments.Sum(c => c.Length);
        m_isClosed = _startPoint.DistanceTo(_endPoint) < Constants.DefaultTolerance;
    }

    /// <summary>
    /// Initializes a new <see cref="GeoPolycurve"/> by connecting a list of ordered <see cref="Point3d"/> points using straight lines.
    /// </summary>
    /// <param name="points">The list of points to connect in order.</param>
    /// <exception cref="ArgumentException">
    /// Thrown if:
    /// <list type="bullet">
    /// <item><description>Less than 2 points are provided.</description></item>
    /// <item><description>All points are identical or too close together.</description></item>
    /// </list>
    /// </exception>
    /// <remarks>
    /// This constructor creates a polycurve consisting only of <see cref="GeoLine"/> segments.
    /// </remarks>
    public GeoPolycurve(List<Point3d> points)
    {
        if (points == null || points.Count < 2)
            throw new ArgumentException("Cần ít nhất 2 điểm để tạo GeoPolycurve.");

        var segments = new List<GeoCurves>();
        for (int i = 0; i < points.Count - 1; i++)
        {
            if (points[i].DistanceTo(points[i + 1]) < Constants.DefaultTolerance)
                continue; // bỏ qua đoạn trùng điểm

            segments.Add(new GeoLine(points[i], points[i + 1]));
        }

        if (segments.Count == 0)
            throw new ArgumentException("Danh sách điểm không hợp lệ (các điểm trùng nhau).");

        Segments = segments;
        Vertices = new List<Point3d>(points);
        _startPoint = points.First();
        _endPoint = points.Last();
        _length = segments.Sum(s => s.Length);
        m_isClosed = _startPoint.DistanceTo(_endPoint) < Constants.DefaultTolerance;
    }

    /// <summary>
    /// Khởi tạo GeoPolycurve từ đối tượng AutoCAD Polyline3d.
    /// Yêu cầu đang nằm trong Transaction đang mở.
    /// </summary>
    /// <param name="polyline3d">Đối tượng Polyline3d đã mở trong transaction hiện tại.</param>
    public GeoPolycurve(Polyline3d polyline3d)
    {
        if (polyline3d == null)
            throw new ArgumentNullException(nameof(polyline3d));

        var pts = new List<Point3d>();

        foreach (ObjectId vId in polyline3d)
        {
            var vertex = vId.GetObject(OpenMode.ForRead) as PolylineVertex3d;
            if (vertex != null)
                pts.Add(vertex.Position);
        }

        if (pts.Count < 2)
            throw new ArgumentException("Polyline3d phải có ít nhất 2 điểm để tạo GeoPolycurve.");

        var segments = new List<GeoCurves>();
        for (int i = 0; i < pts.Count - 1; i++)
        {
            if (pts[i].DistanceTo(pts[i + 1]) > Constants.DefaultTolerance)
                segments.Add(new GeoLine(pts[i], pts[i + 1]));
        }

        // 👇 Thêm đoạn này để xử lý polyline đóng
        bool isClosed = polyline3d.Closed;
        if (isClosed && pts.Last().DistanceTo(pts.First()) > Constants.DefaultTolerance)
        {
            segments.Add(new GeoLine(pts.Last(), pts.First()));
        }

        Segments = segments;
        Vertices = pts;
        _startPoint = pts.First();
        _endPoint = pts.Last();
        _length = segments.Sum(s => s.Length);
        m_isClosed = isClosed;
    }


    #endregion

    #region Method

    /// <summary>
    /// Groups a list of <see cref="GeoCurves"/> into multiple continuous <see cref="GeoPolycurve"/> chains.
    /// </summary>
    /// <param name="geoCurves">A list of unordered curve segments to group.</param>
    /// <returns>
    /// A list of <see cref="GeoPolycurve"/> objects, each representing a connected chain of segments.
    /// </returns>
    /// <remarks>
    /// This method automatically reorders and reverses segments if necessary to form continuous polycurves.
    /// Each chain is built by connecting curve endpoints within the default tolerance.
    /// Segments that cannot be connected to others will form their own individual polycurve.
    /// </remarks>
    public static List<GeoPolycurve> ByGroupCurves(List<GeoCurves> geoCurves)
    {
        if (geoCurves == null || geoCurves.Count == 0)
            return new List<GeoPolycurve>();

        var remaining = new List<GeoCurves>(geoCurves);
        var result = new List<GeoPolycurve>();

        while (remaining.Count > 0)
        {
            var segments = new LinkedList<GeoCurves>();
            var points = new LinkedList<Point3d>();

            var seed = remaining[0];
            remaining.RemoveAt(0);

            segments.AddLast(seed);
            points.AddLast(seed.StartPoint);
            points.AddLast(seed.EndPoint);

            bool extended;

            do
            {
                extended = false;

                for (int i = 0; i < remaining.Count; i++)
                {
                    var c = remaining[i];

                    if (points.Last.Value.IsEqualTo(c.StartPoint))
                    {
                        points.AddLast(c.EndPoint);
                        segments.AddLast(c);
                        remaining.RemoveAt(i);
                        extended = true;
                        break;
                    }

                    if (points.Last.Value.IsEqualTo(c.EndPoint))
                    {
                        c.Reverse(); // đảo tại chỗ, không gán
                        points.AddLast(c.EndPoint);
                        segments.AddLast(c);
                        remaining.RemoveAt(i);
                        extended = true;
                        break;
                    }

                    if (points.First.Value.IsEqualTo(c.EndPoint))
                    {
                        c.Reverse(); // Đảo tại chỗ
                        points.AddFirst(c.StartPoint);
                        segments.AddFirst(c);
                        remaining.RemoveAt(i);
                        extended = true;
                        break;
                    }

                    if (points.First.Value.IsEqualTo(c.StartPoint))
                    {
                        points.AddFirst(c.EndPoint);
                        segments.AddFirst(c);
                        remaining.RemoveAt(i);
                        extended = true;
                        break;
                    }
                }

            } while (extended);

            // Sau khi nối xong 1 nhóm → tạo GeoPolycurve
            var poly = new GeoPolycurve(segments.ToList());
            result.Add(poly);
        }

        return result;
    }

    /// <summary>
    /// Returns basic information about the GeoPolycurve structure.
    /// </summary>
    public string PrintInfo()
    {
        var sb = new StringBuilder();

        sb.AppendLine("===== GeoPolycurve Info =====");
        sb.AppendLine($"IsClosed: {IsClosed}");
        sb.AppendLine($"Total Length: {Length:F4}");
        sb.AppendLine($"Vertex Count: {Vertices.Count}");
        sb.AppendLine($"Segment Count: {Segments.Count}");

        for (int i = 0; i < Vertices.Count; i++)
        {
            var pt = Vertices[i];
            sb.AppendLine($"  Vertex {i + 1}: ({pt.X:F4}, {pt.Y:F4}, {pt.Z:F4})");
        }

        for (int i = 0; i < Segments.Count; i++)
        {
            var seg = Segments[i];
            sb.AppendLine($"  Segment {i + 1}: {seg.GetType().Name}, Length = {seg.Length:F4}");
        }

        return sb.ToString();
    }

    /// <summary>
    /// Performs Ear Clipping triangulation on the GeoPolycurve if it is closed.
    /// </summary>
    public List<GeoTriangle> Triangulate()
    {
        var triangles = new List<GeoTriangle>();

        if (!IsClosed || Vertices.Count < 3)
            return triangles;

        // Step 1: Tạo mặt phẳng chiếu từ 3 điểm đầu tiên
        var plane = new GeoPlane(Vertices[0], Vertices[1], Vertices[2]);
        var projected = Vertices.Select(p => new { Original = p, Flat = plane.ProjectPointTo2D(p) }).ToList();

        var flatPoints = projected.Select(p => p.Flat).ToList();
        var originals = projected.Select(p => p.Original).ToList();

        var indices = Enumerable.Range(0, flatPoints.Count).ToList();

        // Step 2: Đảm bảo polygon theo chiều CCW
        if (GetSignedArea(flatPoints) < 0)
            indices.Reverse();

        int loopGuard = 0; // để tránh vòng lặp vô hạn

        // Step 3: Thuật toán Ear Clipping
        while (indices.Count > 3 && loopGuard < 1000)
        {
            bool earFound = false;

            for (int i = 0; i < indices.Count; i++)
            {
                int i0 = indices[(i - 1 + indices.Count) % indices.Count];
                int i1 = indices[i];
                int i2 = indices[(i + 1) % indices.Count];

                var a = flatPoints[i0];
                var b = flatPoints[i1];
                var c = flatPoints[i2];

                if (!IsConvex(a, b, c))
                    continue;

                // Kiểm tra có điểm nào nằm trong tam giác không
                bool anyInside = false;
                for (int j = 0; j < indices.Count; j++)
                {
                    if (indices[j] == i0 || indices[j] == i1 || indices[j] == i2)
                        continue;

                    if (IsPointInTriangle(flatPoints[indices[j]], a, b, c))
                    {
                        anyInside = true;
                        break;
                    }
                }

                if (!anyInside)
                {
                    // ✅ Đây là một tai, cắt tam giác
                    triangles.Add(new GeoTriangle(
                        originals[i0],
                        originals[i1],
                        originals[i2]
                    ));
                    indices.RemoveAt(i);
                    earFound = true;
                    break;
                }
            }

            if (!earFound)
                break;

            loopGuard++;
        }

        // Còn lại 1 tam giác cuối
        if (indices.Count == 3)
        {
            triangles.Add(new GeoTriangle(
                originals[indices[0]],
                originals[indices[1]],
                originals[indices[2]]
            ));
        }

        return triangles;
    }

    /// <summary>
    /// Computes the minimum distance between a GeoLine and any GeoLine segment in the GeoPolycurve.
    /// </summary>
    public double DistanceTo(GeoLine line)
    {
        double minDist = double.MaxValue;

        foreach (var segment in Segments.OfType<GeoLine>())
        {
            double dist = segment.DistanceTo(line);
            if (dist < minDist)
                minDist = dist;

            // Early return nếu đã chạm
            if (minDist < Constants.DefaultTolerance)
                return 0;
        }

        return minDist;
    }

    #endregion
}

public class GeoTriangle
{
    #region Fields

    private Point3d _p1;
    private Point3d _p2;
    private Point3d _p3;

    #endregion

    #region Properties

    public Point3d P1 { get => _p1; }
    public Point3d P2 { get => _p2; }
    public Point3d P3 { get => _p3; }

    public Point3d[] Vertices => new[] { _p1, _p2, _p3 };

    #endregion

    #region Constructor
    /// <summary>
    /// Tạo một GeoTriangle từ một đối tượng Polyline3d.
    /// Chỉ thành công nếu Polyline3d có đúng 3 đỉnh.
    /// Yêu cầu đang trong một Transaction đang mở.
    /// </summary>
    /// <returns>Một đối tượng GeoTriangle mới, hoặc null nếu không thành công.</returns>
    public static GeoTriangle FromPolyline3d(Polyline3d poly)
    {
        if (poly == null || poly.IsDisposed)
        {
            return null;
        }

        var vertices = new List<Point3d>();
        foreach (ObjectId vertexId in poly)
        {
            if (!vertexId.IsErased)
            {
                var vertex = vertexId.GetObject(OpenMode.ForRead) as PolylineVertex3d;
                if (vertex != null)
                {
                    vertices.Add(vertex.Position);
                }
            }
        }

        // Nếu polyline đóng, đỉnh cuối trùng đỉnh đầu, ta bỏ đi 1 đỉnh.
        if (poly.Closed && vertices.Count == 4 && vertices.First().IsEqualTo(vertices.Last()))
        {
            vertices.RemoveAt(vertices.Count - 1);
        }

        if (vertices.Count == 3)
        {
            return new GeoTriangle(vertices[0], vertices[1], vertices[2]);
        }

        return null;
    }

    /// <summary>
    /// Constructs a GeoTriangle from three 3D points.
    /// </summary>
    /// <param name="p1">First vertex of the triangle</param>
    /// <param name="p2">Second vertex of the triangle</param>
    /// <param name="p3">Third vertex of the triangle</param>
    public GeoTriangle(Point3d p1, Point3d p2, Point3d p3)
    {
        this._p1 = p1;
        this._p2 = p2;
        this._p3 = p3;
    }

    #endregion

    #region Methods

    /// <summary>
    /// Computes the unit normal vector of the triangle (right-hand rule based on vertex order).
    /// </summary>
    /// <returns>Normalized normal vector</returns>
    public Vector3d GetNormal()
    {
        Vector3d v1 = _p2 - _p1;
        Vector3d v2 = _p3 - _p1;
        return v1.CrossProduct(v2).GetNormal();
    }

    public double GetArea()
    {
        Vector3d v1 = _p2 - _p1;
        Vector3d v2 = _p3 - _p1;
        return 0.5 * v1.CrossProduct(v2).Length;
    }

    public Point3d GetCentroid()
    {
        double x = (_p1.X + _p2.X + _p3.X) / 3.0;
        double y = (_p1.Y + _p2.Y + _p3.Y) / 3.0;
        double z = (_p1.Z + _p2.Z + _p3.Z) / 3.0;
        return new Point3d(x, y, z);
    }

    public void Translate(Vector3d vector)
    {
        _p1 += vector;
        _p2 += vector;
        _p3 += vector;
    }

    public void Rotate(Point3d basePoint, Vector3d axis, double angleInRadians)
    {
        var transform = Matrix3d.Rotation(angleInRadians, axis, basePoint);
        _p1 = _p1.TransformBy(transform);
        _p2 = _p2.TransformBy(transform);
        _p3 = _p3.TransformBy(transform);
    }

    public double DistanceTo(Point3d point)
    {
        Vector3d normal = this.GetNormal();
        Vector3d vectorFromPlaneToPoint = point - this._p1;
        return System.Math.Abs(vectorFromPlaneToPoint.DotProduct(normal));
    }

    public bool IsPointInside(Point3d point)
    {
        // Đầu tiên, kiểm tra xem điểm có nằm trên mặt phẳng của tam giác không.
        if (this.DistanceTo(point) > Constants.DefaultTolerance)
        {
            return false;
        }

        // Sử dụng phương pháp tọa độ Barycentric
        Vector3d v0 = _p3 - _p1;
        Vector3d v1 = _p2 - _p1;
        Vector3d v2 = point - _p1;

        double dot00 = v0.DotProduct(v0);
        double dot01 = v0.DotProduct(v1);
        double dot02 = v0.DotProduct(v2);
        double dot11 = v1.DotProduct(v1);
        double dot12 = v1.DotProduct(v2);

        double invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
        double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
        double v = (dot00 * dot12 - dot01 * dot02) * invDenom;

        // Kiểm tra điều kiện
        return (u >= 0) && (v >= 0) && (u + v <= 1);
    }

    public GeoCurves IntersectWith(GeoLine line)
    {
        Vector3d edge1 = this._p2 - this._p1;
        Vector3d edge2 = this._p3 - this._p1;

        Point3d rayOrigin = line.StartPoint;
        Vector3d rayVector = line.Direction;

        Vector3d h = rayVector.CrossProduct(edge2);
        double a = edge1.DotProduct(h);

        // --- TRƯỜNG HỢP 1: ĐỒNG PHẲNG ---
        if (System.Math.Abs(a) < Constants.DefaultTolerance)
        {
            if (this.DistanceTo(rayOrigin) > Constants.DefaultTolerance)
            {
                return null;
            }

            var allPoints = new List<Point3d>();
            var triangleEdges = new[]
            {
                    new GeoLine(this._p1, this._p2),
                    new GeoLine(this._p2, this._p3),
                    new GeoLine(this._p3, this._p1)
                };

            foreach (var edge in triangleEdges)
            {
                var result = line.IntersectWith(edge);
                if (result != null && result.Type == IntersectionResult.IntersectType.Intersect)
                {
                    allPoints.AddRange(result.Points);
                }
            }

            if (this.IsPointInside(line.StartPoint)) allPoints.Add(line.StartPoint);
            if (this.IsPointInside(line.EndPoint)) allPoints.Add(line.EndPoint);

            // --- SỬA LOGIC TẠI ĐÂY ---
            // Dùng vòng lặp và GeometryUtils.Equals để lọc điểm duy nhất
            var distinctPoints = new List<Point3d>();
            foreach (Point3d pt in allPoints)
            {
                if (!distinctPoints.Any(p => GeometryUtils.Equals(p, pt)))
                {
                    distinctPoints.Add(pt);
                }
            }

            if (distinctPoints.Count == 0) return null;
            if (distinctPoints.Count == 1) return new GeoLine(distinctPoints[0], distinctPoints[0]);

            distinctPoints = distinctPoints.OrderBy(p => (p - rayOrigin).DotProduct(rayVector)).ToList();
            return new GeoLine(distinctPoints.First(), distinctPoints.Last());
        }

        // --- TRƯỜNG HỢP 2: ĐÂM XUYÊN ---
        double f = 1.0 / a;
        Vector3d s = rayOrigin - this._p1;
        double u = f * s.DotProduct(h);
        if (u < -Constants.DefaultTolerance || u > 1 + Constants.DefaultTolerance) return null;

        Vector3d q = s.CrossProduct(edge1);
        double v = f * rayVector.DotProduct(q);
        if (v < -Constants.DefaultTolerance || u + v > 1 + Constants.DefaultTolerance) return null;

        double t = f * edge2.DotProduct(q);

        if (t >= -Constants.DefaultTolerance && t <= line.Length + Constants.DefaultTolerance)
        {
            Point3d intersectionPoint = rayOrigin + t * line.Direction.GetNormal();
            return new GeoLine(intersectionPoint, intersectionPoint);
        }

        return null;
    }

    public bool IsCoplanarWith(GeoTriangle other)
    {
        var n1 = this.GetNormal();
        var n2 = other.GetNormal();

        // Bước 1: Vector pháp tuyến phải song song
        if (!n1.IsParallelTo(n2, new Tolerance(Constants.DefaultTolerance, Constants.DefaultTolerance)))
            return false;

        // Bước 2: Khoảng cách từ 1 điểm tam giác này đến mặt phẳng tam giác kia ≈ 0
        double distance = (other.P1 - this.P1).DotProduct(n1);
        return Math.Abs(distance) < Constants.DefaultTolerance;
    }

    public List<GeoCurves> IntersectWith(GeoArc arc)
    {
        var results = new List<GeoCurves>();
        Vector3d planeNormal = this.GetNormal();

        // --- TRƯỜNG HỢP 1: ĐỒNG PHẲNG ---
        if (arc.Normal.IsParallelTo(planeNormal, new Tolerance(Constants.DefaultTolerance, Constants.DefaultTolerance)) &&
            this.DistanceTo(arc.CenterPoint) < Constants.DefaultTolerance)
        {
            var allIntersectionPoints = new List<Point3d>();
            var triangleEdges = new[] {
                    new GeoLine(this._p1, this._p2), new GeoLine(this._p2, this._p3), new GeoLine(this._p3, this._p1)
                };

            foreach (var edge in triangleEdges)
            {
                var intersectionResult = arc.IntersectWith(edge);
                if (intersectionResult.Type == IntersectionResult.IntersectType.Intersect)
                {
                    allIntersectionPoints.AddRange(intersectionResult.Points);
                }
            }

            if (this.IsPointInside(arc.StartPoint)) allIntersectionPoints.Add(arc.StartPoint);
            if (this.IsPointInside(arc.EndPoint)) allIntersectionPoints.Add(arc.EndPoint);

            if (allIntersectionPoints.Count == 0)
            {
                if (this.IsPointInside(arc.StartPoint) && this.IsPointInside(arc.MidPoint))
                {
                    results.Add(arc);
                }
                return results;
            }

            var splitSegments = arc.SplitByPoints(allIntersectionPoints);

            foreach (var segment in splitSegments)
            {
                if (segment is GeoArc subArc)
                {
                    if (this.IsPointInside(subArc.MidPoint))
                    {
                        results.Add(subArc);
                    }
                }
            }
            return results;
        }

        // --- TRƯỜNG HỢP 2: ĐÂM XUYÊN ---
        Point3d planePoint = this._p1;
        Vector3d xAxis = (arc.StartPoint - arc.CenterPoint).GetNormal();
        Vector3d yAxis = arc.Normal.CrossProduct(xAxis);

        double A = arc.Radius * xAxis.DotProduct(planeNormal);
        double B = arc.Radius * yAxis.DotProduct(planeNormal);
        double D = planeNormal.DotProduct(planePoint - arc.CenterPoint);

        double sumSq = A * A + B * B;
        if (sumSq < Constants.DefaultTolerance * Constants.DefaultTolerance) return results;

        double distSq = D * D;
        if (distSq > sumSq + Constants.DefaultTolerance) return results;

        double alpha = Math.Atan2(B, A);
        double temp = Math.Max(-1.0, Math.Min(1.0, D / Math.Sqrt(sumSq)));

        double angle1 = alpha + Math.Acos(temp);
        Point3d pt1 = arc.CenterPoint + arc.Radius * (Math.Cos(angle1) * xAxis + Math.Sin(angle1) * yAxis);

        if (arc.IsPointOnArc(pt1) && this.IsPointInside(pt1))
        {
            results.Add(new GeoLine(pt1, pt1));
        }

        if (distSq < sumSq - Constants.DefaultTolerance)
        {
            double angle2 = alpha - Math.Acos(temp);
            Point3d pt2 = arc.CenterPoint + arc.Radius * (Math.Cos(angle2) * xAxis + Math.Sin(angle2) * yAxis);

            if (arc.IsPointOnArc(pt2) && this.IsPointInside(pt2) && !GeometryUtils.Equals(pt1, pt2))
            {
                results.Add(new GeoLine(pt2, pt2));
            }
        }

        return results;
    }

    public bool IsParallelTo(GeoTriangle other)
    {
        if (other == null) return false;

        Vector3d normal1 = this.GetNormal();
        Vector3d normal2 = other.GetNormal();

        // Nếu tích có hướng bằng 0 thì song song hoặc trùng
        return normal1.IsParallelTo(normal2, new Tolerance(Constants.DefaultTolerance, Constants.DefaultTolerance));
    }

    public GeoLine GetIntersectionLineWith(GeoTriangle other)
    {
        if (this.IsCoplanarWith(other) || this.IsParallelTo(other))
            return null;

        Vector3d n1 = this.GetNormal();
        Vector3d n2 = other.GetNormal();
        Vector3d direction = n1.CrossProduct(n2).GetNormal();

        // Mặt phẳng 1: n1 . (P - A1) = 0 → n1.X*x + n1.Y*y + n1.Z*z = d1
        // Mặt phẳng 2: n2 . (P - A2) = 0 → n2.X*x + n2.Y*y + n2.Z*z = d2

        double d1 = n1.DotProduct(this.P1 - Point3d.Origin);
        double d2 = n2.DotProduct(other.P1 - Point3d.Origin);

        // Chọn 1 biến tự do (ưu tiên z nếu direction.Z ≠ 0)
        double x, y, z;
        if (Math.Abs(direction.Z) > Constants.DefaultTolerance)
        {
            z = 0; // fix z
            var a1 = n1.X;
            var b1 = n1.Y;
            var c1 = n1.Z;
            var a2 = n2.X;
            var b2 = n2.Y;
            var c2 = n2.Z;

            double denom = a1 * b2 - a2 * b1;
            if (Math.Abs(denom) < Constants.DefaultTolerance)
                return null;

            x = (d1 * b2 - d2 * b1) / denom;
            y = (a1 * d2 - a2 * d1) / denom;
        }
        else if (Math.Abs(direction.Y) > Constants.DefaultTolerance)
        {
            y = 0;
            double a1 = n1.X, c1 = n1.Z;
            double a2 = n2.X, c2 = n2.Z;

            double denom = a1 * c2 - a2 * c1;
            if (Math.Abs(denom) < Constants.DefaultTolerance)
                return null;

            x = (d1 * c2 - d2 * c1) / denom;
            z = (a1 * d2 - a2 * d1) / denom;
        }
        else
        {
            x = 0;
            double b1 = n1.Y, c1 = n1.Z;
            double b2 = n2.Y, c2 = n2.Z;

            double denom = b1 * c2 - b2 * c1;
            if (Math.Abs(denom) < Constants.DefaultTolerance)
                return null;

            y = (d1 * c2 - d2 * c1) / denom;
            z = (b1 * d2 - b2 * d1) / denom;
        }

        var basePoint = new Point3d(x, y, z);

        // Chiếu tất cả đỉnh của 2 tam giác lên direction vector
        var allPoints = this.Vertices.Concat(other.Vertices).ToList();
        var projections = allPoints.Select(pt => (pt - basePoint).DotProduct(direction)).ToList();

        double tMin = projections.Min();
        double tMax = projections.Max();

        var p1 = basePoint + tMin * direction;
        var p2 = basePoint + tMax * direction;

        return new GeoLine(p1, p2);
    }

    public GeoLine GetIntersection(GeoTriangle other)
    {
        // Bước 1: Tìm đường giao tuyến giữa 2 mặt phẳng tam giác
        GeoLine intersectLine = this.GetIntersectionLineWith(other);
        if (intersectLine == null)
            return null;

        // Bước 2: Tìm đoạn cắt với tam giác thứ nhất
        var segment1 = this.IntersectWith(intersectLine) as GeoLine;
        if (segment1 == null)
            return null;

        // Bước 3: Tìm đoạn cắt với tam giác thứ hai
        var segment2 = other.IntersectWith(intersectLine) as GeoLine;
        if (segment2 == null)
            return null;

        // Bước 4: Tìm đoạn giao nhau giữa 2 đoạn con (overlap)
        var overlapResult = segment1.IntersectWith(segment2);
        if (overlapResult.Type == IntersectionResult.IntersectType.Overlap &&
            overlapResult.OverlapSegments != null && overlapResult.OverlapSegments.Count > 0)
        {
            return overlapResult.OverlapSegments[0] as GeoLine;
        }

        return null;
    }

    public Entity[] Test()
    {
        var pts = new Point3dCollection
    {
        _p1,
        _p2,
        _p3,
        _p1 // Đóng kín tam giác
    };

        var polyline = new Polyline3d(Poly3dType.SimplePoly, pts, true)
        {
            ColorIndex = 2 // Màu tùy chọn, ví dụ: xanh lá
        };

        return new Entity[] { polyline };
    }

    #endregion
}