using System;
using System.Collections.Generic;
using System.Linq;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.Geometry;

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

    #endregion

    #region Method
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

    public GeoLine(Point3d start, Point3d end)
    {
        _startPoint = start;
        _endPoint = end;
        _direction = (end - start).GetNormal();
        _length = start.DistanceTo(end);
    }

    public GeoLine(Line line)
    {
        _startPoint = line.StartPoint;
        _endPoint = line.EndPoint;
        _direction = (_endPoint - _startPoint).GetNormal();
        _length = _startPoint.DistanceTo(_endPoint);
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
    private IntersectionResult IntersectWithArc(GeoArc arc)
    {
        var p = StartPoint;
        var q = EndPoint;
        var d = q - p;

        var planeNormal = arc.Normal;
        var planePoint = arc.CenterPoint;

        var denom = d.DotProduct(planeNormal);
        if (Math.Abs(denom) < Constants.DefaultTolerance)
            return IntersectionResult.None(); // Line is parallel to arc plane

        var t = (planePoint - p).DotProduct(planeNormal) / denom;
        if (t < -Constants.DefaultTolerance || t > 1 + Constants.DefaultTolerance)
            return IntersectionResult.None(); // Intersection is outside line segment

        var intersection = p + t * d;

        // Check if the point lies on the arc sweep
        var v = (intersection - arc.CenterPoint).GetNormal();
        var vStart = (arc.StartPoint - arc.CenterPoint).GetNormal();
        var angleTo = GetAngleOnPlane(vStart, v, arc.Normal);

        if (angleTo < -Constants.DefaultTolerance || angleTo > arc.Angle + Constants.DefaultTolerance)
            return IntersectionResult.None();

        return IntersectionResult.Intersect(intersection);
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

    public GeoArc(Arc arc)
    {
        if (arc == null)
            throw new ArgumentNullException(nameof(arc));

        // Lấy các điểm đặc trưng
        m_startpoint = arc.StartPoint;
        m_endpoint = arc.EndPoint;
        m_midPoint = arc.GetPointAtDist(arc.Length / 2.0);

        // Dựng lại từ 3 điểm
        var reconstructed = new GeoArc(m_startpoint, m_midPoint, m_endpoint);

        // Gán toàn bộ dữ liệu hình học
        m_centerPoint = reconstructed.CenterPoint;
        m_normal = reconstructed.Normal;
        m_plane = reconstructed.Plane;
        m_radius = reconstructed.Radius;
        m_angle = reconstructed.Angle;
        m_length = reconstructed.Length;
        m_startAngle = reconstructed.StartAngle;
        m_endAngle = reconstructed.EndAngle;
        m_isClosed = reconstructed.IsClosed;
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
    private IntersectionResult IntersectWithArc(GeoArc other)
    {
        // Check coplanarity
        if ((this.CenterPoint - other.CenterPoint).DotProduct(this.Normal.CrossProduct(other.Normal)) > Constants.DefaultTolerance)
            return IntersectionResult.None();

        var r1 = this.Radius;
        var r2 = other.Radius;
        var c1 = this.CenterPoint;
        var c2 = other.CenterPoint;
        var d = c2 - c1;
        double dist = d.Length;

        // No intersection (too far apart or one inside the other)
        if (dist > r1 + r2 + Constants.DefaultTolerance || dist < Math.Abs(r1 - r2) - Constants.DefaultTolerance)
            return IntersectionResult.None();

        // Circle-circle intersection (in 2D)
        var a = (r1 * r1 - r2 * r2 + dist * dist) / (2 * dist);
        var h = Math.Sqrt(Math.Max(0, r1 * r1 - a * a));

        var p2 = c1 + a * d.GetNormal();
        var offset = d.GetNormal().CrossProduct(this.Normal).GetNormal();

        var i1 = p2 + h * offset;
        var i2 = p2 - h * offset;

        var results = new List<Point3d>();

        if (IsPointOnArc(this, i1) && IsPointOnArc(other, i1)) results.Add(i1);
        if (i2.DistanceTo(i1) > Constants.DefaultTolerance)
        {
            if (IsPointOnArc(this, i2) && IsPointOnArc(other, i2)) results.Add(i2);
        }

        if (results.Count == 0) return IntersectionResult.None();
        if (results.Count == 1) return IntersectionResult.Intersect(results[0]);
        return IntersectionResult.Intersect(results);
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

        Segments = segments;
        Vertices = pts;
        _startPoint = pts.First();
        _endPoint = pts.Last();
        _length = segments.Sum(s => s.Length);
        m_isClosed = _startPoint.DistanceTo(_endPoint) < Constants.DefaultTolerance;
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

    #endregion
}

public class GeoFace
{
    #region Fields

    #endregion
}