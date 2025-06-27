using System;
using System.Collections.Generic;
using System.Linq;
using System.Security.Cryptography;
using System.Text;
using System.Threading.Tasks;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.Geometry;
using static GeoLine;


public static class Constants
{
    public const double DefaultTolerance = 1e-9;
    public const int MaxRetries = 3;
    public const string DefaultUserName = "Guest";
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
    public GeoCoordinateSystem(Point3d origin, Vector3d xAxis, Vector3d yAxis, Vector3d zAxis)
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

public abstract class GeoCurves
{

    #region Public Properties
    public abstract Point3d StartPoint
    {
        get;
        protected set;
    }
    public abstract Point3d EndPoint
    {
        get;
        protected set;
    }
    public abstract bool IsClosed
    {
        get;
        protected set;
    }
    public abstract double Length
    {
        get;
        protected set;
    }
    #endregion

    #region Public Abtract Methods

    /// <summary>
    /// Lấy hệ trục tọa độ (local coordinate system) tại tham số t.
    /// Dùng để xác định mặt cắt vuông góc tại vị trí bất kỳ trên đường.
    /// </summary>
    /// <param name="t">Giá trị tham số từ 0 đến 1 dọc theo chiều dài đường cong.</param>
    /// <returns>Hệ tọa độ tại tham số đó.</returns>
    public abstract GeoCoordinateSystem CoordinateSystemAtParameter(double t);

    /// <summary>
    /// Lấy hệ trục tọa độ tại vị trí cách điểm đầu một đoạn có chiều dài t (tính theo mét).
    /// Khác với CoordinateSystemAtParameter, tham số ở đây là chiều dài tuyệt đối.
    /// </summary>
    /// <param name="t">Chiều dài tính từ điểm đầu (tính bằng mét).</param>
    /// <returns>Hệ tọa độ tại vị trí đó.</returns>
    public abstract GeoCoordinateSystem CoordinateSystemAtSegmentLength(double t);

    /// <summary>
    /// Tính vector pháp tuyến tại vị trí t.
    /// Thường sử dụng trong dựng mặt phẳng vuông góc hoặc cho visualization.
    /// </summary>
    /// <param name="t">Giá trị tham số từ 0 đến 1.</param>
    /// <returns>Vector pháp tuyến tại vị trí đó.</returns>
    public abstract Vector3d NormalAtParameter(double t);

    /// <summary>
    /// Tính tham số t (từ 0 đến 1) tương ứng với một điểm cụ thể trên đường.
    /// Nếu điểm không nằm trên đường thì vẫn trả về tham số gần nhất theo chiếu vuông góc.
    /// </summary>
    /// <param name="point">Điểm bất kỳ trên không gian.</param>
    /// <returns>Giá trị t (0 ≤ t ≤ 1) dọc theo chiều dài đường cong.</returns>
    public abstract double ParameterAtPoint(Point3d point);

    /// <summary>
    /// Tạo một mặt phẳng đi qua điểm tại tham số t, với vector pháp tuyến xác định bởi hướng tiếp tuyến tại t.
    /// </summary>
    /// <param name="t">Tham số vị trí (0 đến 1).</param>
    /// <returns>Mặt phẳng vuông góc tại điểm đó.</returns>
    public abstract GeoPlane PlaneAtParameter(double t);

    /// <summary>
    /// Trả về tọa độ điểm tương ứng với tham số t.
    /// </summary>
    /// <param name="t">Tham số trong đoạn [0,1].</param>
    /// <returns>Điểm trên đường cong tương ứng với t.</returns>
    public abstract Point3d PointAtParameter(double t);

    /// <summary>
    /// Chiếu toàn bộ đường cong xuống một mặt phẳng bất kỳ.
    /// </summary>
    /// <param name="plane">Mặt phẳng đích cần chiếu xuống.</param>
    /// <returns>Đường cong mới sau khi chiếu lên mặt phẳng.</returns>
    public abstract GeoCurves PullOntoPlane(GeoPlane plane);

    /// <summary>
    /// Đảo ngược hướng đi của đường cong (điểm đầu thành điểm cuối và ngược lại).
    /// </summary>
    /// <returns>Đường cong mới theo hướng ngược lại.</returns>
    public abstract GeoCurves Reverse();

    /// <summary>
    /// Cắt đường cong thành hai phần tại tham số t = 0.5 (giữa đường).
    /// </summary>
    /// <returns>Đoạn đầu tiên sau khi cắt, kiểu AutoCAD Curve.</returns>
    public abstract List<GeoCurves> SplitByParameter(List<double> parameters);

    /// <summary>
    /// Cắt đường cong tại điểm trung gian (giả lập), thường dùng để minh họa.
    /// </summary>
    /// <returns>Đoạn đầu tiên sau khi cắt, kiểu AutoCAD Curve.</returns>
    public abstract List<GeoCurves> SplitByPoints(List<Point3d> points);

    public abstract double DistanceTo(Point3d point);
    #endregion
}

public sealed class GeoLine: GeoCurves
{
    #region Fields
    private Point3d _startPoint;
    private Point3d _endPoint;
    private Vector3d _direction;
    private bool m_isClosed;
    private double _length;
    #endregion

    #region Properties
    public override Point3d StartPoint { get => _startPoint; protected set => _startPoint = value; }
    public override Point3d EndPoint { get => _endPoint; protected set => _endPoint = value; }
    public override bool IsClosed { get => m_isClosed; protected set => m_isClosed = value; }
    public override double Length { get => _length; protected set => _length = value; }

    public Vector3d Direction => _direction;
    #endregion

    #region Constructor
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
    public override Point3d PointAtParameter(double t)
    {
        return StartPoint + t * (EndPoint - StartPoint); 
    }

    public override double ParameterAtPoint(Point3d point)
    {
        var lineVec = EndPoint - StartPoint;
        var toPoint = point - StartPoint;
        return toPoint.DotProduct(lineVec) / lineVec.LengthSqrd;
    }

    public override GeoCoordinateSystem CoordinateSystemAtParameter(double t)
    {
        var origin = PointAtParameter(t);
        var z = Direction;

        Vector3d x = Math.Abs(z.DotProduct(Vector3d.XAxis)) > 0.9 ? Vector3d.YAxis : Vector3d.XAxis;
        Vector3d y = z.CrossProduct(x).GetNormal();
        x = y.CrossProduct(z).GetNormal();
        return new GeoCoordinateSystem(origin, x, y, z);
    }

    public override GeoCoordinateSystem CoordinateSystemAtSegmentLength(double t)
    {
        double param = t / Length;
        return CoordinateSystemAtParameter(param);
    }

    public override Vector3d NormalAtParameter(double t)
    {
        Vector3d worldZ = Vector3d.ZAxis;

        // Nếu tangent gần song song với Z, chọn trục khác
        if (Math.Abs(Direction.DotProduct(worldZ)) > 0.99)
            worldZ = Vector3d.YAxis;

        // Pháp tuyến mặt phẳng chứa đoạn thẳng theo hướng Z
        Vector3d normal = Direction.CrossProduct(worldZ).GetNormal();

        return normal;
    }

    public override GeoPlane PlaneAtParameter(double t)
    {
        var origin = PointAtParameter(t);
        var normal = NormalAtParameter(t);
        return new GeoPlane(origin, normal);
    }

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

    public override GeoCurves Reverse()
    {
        return new GeoLine(EndPoint, StartPoint);
    }

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

        // Thêm đoạn cuối (t -> 1)
        if (prevT < 1.0 - 1e-6)
        {
            Point3d pt1 = PointAtParameter(prevT);
            Point3d pt2 = PointAtParameter(1.0);
            result.Add(new GeoLine(pt1, pt2));
        }

        return result;
    }

    public override List<GeoCurves> SplitByPoints(List<Point3d> points)
    {
        if (points == null || points.Count == 0)
            return new List<GeoCurves> { this };

        var parameters = points
            .Select(p => ParameterAtPoint(p))
            .ToList();

        return SplitByParameter(parameters);
    }

    public override double DistanceTo(Point3d point)
    {
        Vector3d v = EndPoint - StartPoint;
        Vector3d w = point - StartPoint;

        double proj = w.DotProduct(v) / v.LengthSqrd;

        // Clamp thủ công
        if (proj < 0) proj = 0;
        else if (proj > 1) proj = 1;

        Point3d closest = StartPoint + proj * v;
        return point.DistanceTo(closest);
    }
    #endregion

    #region Public Function
    public IntersectType IntersectWith(GeoLine other, out object result)
    {
        result = null;

        Vector3d u = this.Direction;
        Vector3d v = other.Direction;
        Vector3d w0 = this.StartPoint - other.StartPoint;
        Vector3d n = u.CrossProduct(v);

        // Kiểm tra xem hai đường thẳng có song song hoặc trùng nhau không
        if (n.LengthSqrd < Constants.DefaultTolerance)
        {
            // Nếu w0 cũng song song với u, thì hai đường thẳng là trùng nhau (collinear)
            if (w0.CrossProduct(u).LengthSqrd < Constants.DefaultTolerance)
            {
                return HandleCollinearLines(other, u, w0, out result);
            }
            return IntersectType.None; // Song song nhưng không trùng nhau
        }

        // Xử lý trường hợp không song song (có thể cắt nhau)
        return HandleIntersectingLines(other, u, v, w0, out result);
    }

    #endregion

    #region Private Funciton

    /// <summary>
    /// Xử lý trường hợp hai đường thẳng trùng nhau (collinear).
    /// </summary>
    private IntersectType HandleCollinearLines(GeoLine other, Vector3d u, Vector3d w0, out object result)
    {
        result = null;

        double t1 = w0.DotProduct(u) / u.LengthSqrd;
        double t2 = (other.EndPoint - this.StartPoint).DotProduct(u) / u.LengthSqrd;

        double tMin = Math.Max(0, Math.Min(t1, t2));
        double tMax = Math.Min(1, Math.Max(t1, t2));

        // Kiểm tra xem có đoạn trùng lặp nào không
        if (tMin <= tMax + Constants.DefaultTolerance)
        {
            Point3d pt1 = this.StartPoint + u * tMin;
            Point3d pt2 = this.StartPoint + u * tMax;

            if (pt1.DistanceTo(pt2) < Constants.DefaultTolerance)
                result = pt1;
            else
                result = new GeoLine(pt1, pt2);

            return IntersectType.Overlap;
        }

        // Nếu trùng nhau nhưng không chồng lấp đoạn nào
        result = new GeoLine(this.StartPoint, this.EndPoint);
        return IntersectType.SameLine;
    }

    /// <summary>
    /// Xử lý trường hợp hai đường thẳng có thể cắt nhau.
    /// </summary>
    private IntersectType HandleIntersectingLines(GeoLine other, Vector3d u, Vector3d v, Vector3d w0, out object result)
    {
        result = null;

        double a = u.DotProduct(u), b = u.DotProduct(v), c = v.DotProduct(v);
        double d = u.DotProduct(w0), e = v.DotProduct(w0);
        double denom = a * c - b * b;

        // Denom gần bằng 0 có nghĩa là các đường thẳng song song (hoặc quá gần song song),
        // trường hợp này lẽ ra đã được bắt bởi n.LengthSqrd trước đó.
        if (Math.Abs(denom) < Constants.DefaultTolerance)
            return IntersectType.None;

        double s = (b * e - c * d) / denom;
        double t = (a * e - b * d) / denom;

        // Kiểm tra xem giao điểm có nằm trong cả hai đoạn thẳng không
        if (s >= -Constants.DefaultTolerance && s <= 1 + Constants.DefaultTolerance &&
            t >= -Constants.DefaultTolerance && t <= 1 + Constants.DefaultTolerance)
        {
            Point3d p1 = this.StartPoint + s * u;
            Point3d p2 = other.StartPoint + t * v;

            if (p1.DistanceTo(p2) < Constants.DefaultTolerance)
            {
                result = p1;
                return IntersectType.Intersect;
            }
        }

        return IntersectType.None;
    }

    #endregion

    #region Enum
    public enum IntersectType
    {
        None,
        Intersect,
        Overlap,
        SameLine
    }
    #endregion
}

public sealed class GeoArc : GeoCurves
{
    #region Field
    private Point3d m_startpoint;
    private Point3d m_endpoint;
    private Point3d m_midPoint;
    private Point3d m_centerPoint;

    private Vector3d m_normal;

    private GeoPlane m_plane;

    private bool m_isClosed;

    private double m_radius;
    private double m_startAngle; 
    private double m_endAngle;   
    private double m_angle;
    private double m_length;

    #endregion

    #region Properties
    public override Point3d StartPoint { get => m_startpoint; protected set => m_startpoint = value; }
    public override Point3d EndPoint { get => m_endpoint; protected set => m_endpoint = value; } // Lưu ý: `m_startpoint` ở đây là lỗi gõ, phải là `m_endpoint`
    public override bool IsClosed { get => m_isClosed; protected set => m_isClosed = value; }
    public override double Length { get => m_length; protected set => m_length = value; }

    public Point3d MidPoint { get => m_midPoint; set => m_midPoint = value; }
    public Point3d CenterPoint { get => m_centerPoint; protected set => m_centerPoint = value; } // Nên là protected set sau khi khởi tạo
    public GeoPlane Plane { get => m_plane; protected set => m_plane = value; } // Nên là protected set sau khi khởi tạo
    public Vector3d Normal { get => m_normal; protected set => m_normal = value; } // Nên là protected set sau khi khởi tạo
    public double Radius { get => m_radius; protected set => m_radius = value; } // Nên là protected set sau khi khởi tạo
    public double Angle { get => m_angle; protected set => m_angle = value; } // Góc quét
    public double StartAngle { get => m_startAngle; protected set => m_startAngle = value; }
    public double EndAngle { get => m_endAngle; protected set => m_endAngle = value; }

    #endregion

    #region Constructors
    /// <summary>
    /// Hàm khởi tạo GeoArc từ 3 điểm: điểm đầu, điểm giữa, điểm cuối.
    /// </summary>
    /// <param name="startPoint">Điểm bắt đầu của cung.</param>
    /// <param name="midPoint">Điểm bất kỳ nằm trên cung (giữa điểm đầu và điểm cuối).</param>
    /// <param name="endPoint">Điểm kết thúc của cung.</param>
    public GeoArc(Point3d startPoint, Point3d midPoint, Point3d endPoint)
    {
        m_startpoint = startPoint;
        m_midPoint = midPoint;
        m_endpoint = endPoint;

        // Vector hướng
        Vector3d v1 = midPoint - startPoint;
        Vector3d v2 = endPoint - midPoint;

        // Pháp tuyến mặt phẳng chứa cung tròn
        m_normal = v1.CrossProduct(v2).GetNormal();
        if (m_normal.Length < Constants.DefaultTolerance)
            throw new ArgumentException("3 điểm thẳng hàng, không tạo được cung tròn.");

        // Mặt phẳng chứa cung
        m_plane = new GeoPlane(startPoint, m_normal);

        // Trung điểm mỗi đoạn
        Point3d mid1 = startPoint + 0.5 * v1;
        Point3d mid2 = midPoint + 0.5 * v2;

        // Vector trung trực vuông góc với đoạn gốc, nằm trong mặt phẳng
        Vector3d perp1 = v1.CrossProduct(m_normal).GetNormal();
        Vector3d perp2 = v2.CrossProduct(m_normal).GetNormal();

        // Tạo 2 trung trực
        GeoLine bisector1 = new GeoLine(mid1, mid1 + perp1);
        GeoLine bisector2 = new GeoLine(mid2, mid2 + perp2);

        // Tìm tâm đường tròn bằng giao 2 trung trực
        object result;
        var type = bisector1.IntersectWith(bisector2, out result);

        if (type != IntersectType.Intersect || !(result is Point3d center))
            throw new ArgumentException("Không thể xác định tâm cung tròn từ 3 điểm.");

        m_centerPoint = center;

        // Tính bán kính
        m_radius = center.DistanceTo(startPoint);

        // Tính góc
        Vector3d from = (startPoint - center).GetNormal();
        Vector3d to = (endPoint - center).GetNormal();
        m_angle = Vector3dAngleOnPlane(from, to, m_normal);
        m_length = m_angle * m_radius;

        m_startAngle = 0;
        m_endAngle = m_angle;

        m_isClosed = false; // Vì là cung
    }
    #endregion

    #region Private Funciton
    private double Vector3dAngleOnPlane(Vector3d from, Vector3d to, Vector3d planeNormal)
    {
        double angle = from.GetAngleTo(to);
        Vector3d cross = from.CrossProduct(to);
        return cross.DotProduct(planeNormal) < 0 ? 2 * Math.PI - angle : angle;
    }
    #endregion

    #region Override
    public override GeoCoordinateSystem CoordinateSystemAtParameter(double t)
    {
        Point3d origin = PointAtParameter(t);
        Vector3d tangent = TangentAtParameter(t);
        Vector3d z = m_normal; // Pháp tuyến cố định của cung

        Vector3d y = z.CrossProduct(tangent).GetNormal();
        Vector3d x = tangent.GetNormal();

        return new GeoCoordinateSystem(origin, x, y, z);
    }
    public override Point3d PointAtParameter(double t)
    {
        double angle = t * m_angle;

        // Vector từ tâm đến điểm bắt đầu
        Vector3d startVec = (m_startpoint - m_centerPoint).GetNormal();
        Vector3d orthoVec = m_normal.CrossProduct(startVec).GetNormal();

        // Vị trí điểm tương ứng với góc = R(cosθ * X + sinθ * Y)
        Vector3d offset = Math.Cos(angle) * startVec + Math.Sin(angle) * orthoVec;

        return m_centerPoint + m_radius * offset;
    }
    public override GeoCoordinateSystem CoordinateSystemAtSegmentLength(double t)
    {
        double param = t / m_length; // Chuyển từ chiều dài → tham số 0..1

        Point3d origin = PointAtParameter(param);

        Vector3d tangent = TangentAtParameter(param); // = đạo hàm vị trí
        Vector3d z = m_normal;
        Vector3d y = z.CrossProduct(tangent).GetNormal();
        Vector3d x = tangent;

        return new GeoCoordinateSystem(origin, x, y, z);
    }
    public override Vector3d NormalAtParameter(double t)
    {
        double angle = t * m_angle;

        Vector3d X = (m_startpoint - m_centerPoint).GetNormal();
        Vector3d Y = m_normal.CrossProduct(X).GetNormal();

        Vector3d normal = -Math.Cos(angle) * X - Math.Sin(angle) * Y;

        return normal.GetNormal();
    }
    public override double ParameterAtPoint(Point3d point)
    {
        Vector3d from = (point - m_centerPoint).GetNormal();
        Vector3d startVec = (m_startpoint - m_centerPoint).GetNormal();
        Vector3d orthoVec = m_normal.CrossProduct(startVec).GetNormal();

        double cos = from.DotProduct(startVec);
        double sin = from.DotProduct(orthoVec);

        double angle = Math.Atan2(sin, cos);
        if (angle < 0)
            angle += 2 * Math.PI;

        // Chuyển angle về [0, m_angle]
        double t = angle / m_angle;
        return Math.Max(0, Math.Min(1, t));
    }
    public override GeoPlane PlaneAtParameter(double t)
    {
        Point3d origin = PointAtParameter(t);
        Vector3d normal = NormalAtParameter(t);
        return new GeoPlane(origin, normal);
    }
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
    public override GeoCurves Reverse()
    {
        return new GeoArc(m_endpoint, m_midPoint, m_startpoint);
    }
    public override List<GeoCurves> SplitByParameter(List<double> parameters)
    {
        var result = new List<GeoCurves>();

        if (parameters == null || parameters.Count == 0)
        {
            result.Add(this);
            return result;
        }

        var sortedParams = parameters
            .Select(p => Math.Max(0, Math.Min(1, p))) // Clamp
            .Distinct()
            .OrderBy(p => p)
            .ToList();

        double prevT = 0;

        foreach (var t in sortedParams)
        {
            if (Math.Abs(t - prevT) < 1e-6) continue;

            Point3d pt1 = PointAtParameter(prevT);
            Point3d pt2 = PointAtParameter(t);
            Point3d mid = PointAtParameter((prevT + t) / 2.0);

            result.Add(new GeoArc(pt1, mid, pt2));

            prevT = t;
        }

        // Thêm đoạn cuối nếu chưa đủ
        if (prevT < 1.0 - 1e-6)
        {
            Point3d pt1 = PointAtParameter(prevT);
            Point3d pt2 = PointAtParameter(1.0);
            Point3d mid = PointAtParameter((prevT + 1.0) / 2.0);

            result.Add(new GeoArc(pt1, mid, pt2));
        }

        return result;
    }
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
    public override double DistanceTo(Point3d point)
    {
        Vector3d from = (StartPoint - CenterPoint).GetNormal();
        Vector3d ortho = Normal.CrossProduct(from).GetNormal();
        Vector3d to = point - CenterPoint;

        double x = to.DotProduct(from);
        double y = to.DotProduct(ortho);

        double angle = Math.Atan2(y, x);
        if (angle < 0) angle += 2 * Math.PI;

        // Clamp angle trong giới hạn cung
        if (angle > Angle) angle = Angle;

        double t = angle / Angle;
        Point3d closest = PointAtParameter(t);
        return point.DistanceTo(closest);
    }
    #endregion

    #region Public Function
    public Vector3d TangentAtParameter(double t)
    {
        double angle = t * m_angle;

        Vector3d startVec = (m_startpoint - m_centerPoint).GetNormal();
        Vector3d orthoVec = m_normal.CrossProduct(startVec).GetNormal();

        // Đạo hàm của vị trí theo góc là tiếp tuyến: (-sin * X + cos * Y)
        Vector3d tangent = -Math.Sin(angle) * startVec + Math.Cos(angle) * orthoVec;
        return tangent.GetNormal();
    }
    #endregion
}

public sealed class GeoPolycurve: GeoCurves
{
    #region Fields
    private Point3d _startPoint;
    private Point3d _endPoint;
    private bool m_isClosed;
    private double _length;

    private List<GeoCurves> segments;
    private List<Point3d> vertices;
    #endregion

    #region Properties
    public override Point3d StartPoint { get => _startPoint; protected set => _startPoint = value; }
    public override Point3d EndPoint { get => _endPoint; protected set => _endPoint = value; }
    public override bool IsClosed { get => m_isClosed; protected set => m_isClosed = value; }
    public override double Length { get => _length; protected set => _length = value; }

    public List<GeoCurves> Segments { get => segments; set => segments = value; }
    public List<Point3d> Vertices { get => vertices; set => vertices = value; }
    #endregion

    #region Private
    /// <summary>
    /// Từ tham số t (0..1) trên toàn bộ polycurve, xác định đoạn tương ứng và t cục bộ trong đoạn đó.
    /// </summary>
    /// 
    private (int index, double localT) GetSegmentAndLocalT(double t)
    {
        if (Segments == null || Segments.Count == 0)
            throw new InvalidOperationException("GeoPolycurve không có đoạn con nào.");

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

        // Nếu t gần 1 hoặc vượt quá do sai số, trả về đoạn cuối cùng
        return (Segments.Count - 1, 1.0);
    }

    private (int index, double localT) GetSegmentAndLocalTByLength(double absLength)
    {
        if (Segments == null || Segments.Count == 0)
            throw new InvalidOperationException("GeoPolycurve không có đoạn con nào.");

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

        return (Segments.Count - 1, 1.0);
    }
    #endregion

    #region Override 

    public override GeoCoordinateSystem CoordinateSystemAtParameter(double t)
    {
        var (index, localT) = GetSegmentAndLocalT(t);
        return Segments[index].CoordinateSystemAtParameter(localT);
    }

    public override GeoCoordinateSystem CoordinateSystemAtSegmentLength(double t)
    {
        var (index, localT) = GetSegmentAndLocalTByLength(t);
        return Segments[index].CoordinateSystemAtParameter(localT);
    }

    public override Vector3d NormalAtParameter(double t)
    {
        if (Segments == null || Segments.Count == 0)
            throw new InvalidOperationException("GeoPolycurve không có đoạn con nào.");

        var (index, localT) = GetSegmentAndLocalT(t);
        return Segments[index].NormalAtParameter(localT);
    }

    public override double ParameterAtPoint(Point3d point)
    {
        if (Segments == null || Segments.Count == 0)
            throw new InvalidOperationException("GeoPolycurve không có đoạn con nào.");

        double accumulatedLength = 0;
        double totalLength = Length;

        double bestT = 0;
        double bestDistance = double.MaxValue;

        for (int i = 0; i < Segments.Count; i++)
        {
            var segment = Segments[i];
            double localT = segment.ParameterAtPoint(point);
            Point3d closestPt = segment.PointAtParameter(localT);
            double distance = closestPt.DistanceTo(point);

            if (distance < bestDistance)
            {
                bestDistance = distance;

                double segmentLength = segment.Length;
                double globalLengthAtPoint = accumulatedLength + localT * segmentLength;

                bestT = globalLengthAtPoint / totalLength;
            }

            accumulatedLength += segment.Length;
        }

        return bestT;
    }

    public override GeoPlane PlaneAtParameter(double t)
    {
        if (Segments == null || Segments.Count == 0)
            throw new InvalidOperationException("GeoPolycurve không có đoạn con nào.");

        var (index, localT) = GetSegmentAndLocalT(t);
        return Segments[index].PlaneAtParameter(localT);
    }

    public override double DistanceTo(Point3d point)
    {
        double minDist = double.MaxValue;

        foreach (var seg in Segments)
        {
            double d = seg.DistanceTo(point);
            if (d < minDist)
                minDist = d;
        }

        return minDist;
    }

    public override Point3d PointAtParameter(double t)
    {
        if (Segments == null || Segments.Count == 0)
            throw new InvalidOperationException("GeoPolycurve không có đoạn con nào.");

        var (index, localT) = GetSegmentAndLocalT(t);
        return Segments[index].PointAtParameter(localT);
    }

    public override GeoCurves PullOntoPlane(GeoPlane plane)
    {
        var projectedSegments = new List<GeoCurves>();
        foreach (var seg in Segments)
        {
            projectedSegments.Add(seg.PullOntoPlane(plane));
        }

        var projectedVertices = projectedSegments.Select(s => s.StartPoint).ToList();
        projectedVertices.Add(projectedSegments.Last().EndPoint);

        return new GeoPolycurve
        {
            Segments = projectedSegments,
            Vertices = projectedVertices,
            StartPoint = projectedSegments.First().StartPoint,
            EndPoint = projectedSegments.Last().EndPoint,
            IsClosed = this.IsClosed,
            Length = projectedSegments.Sum(s => s.Length)
        };
    }

    public override GeoCurves Reverse()
    {
        var reversedSegments = Segments
            .Select(s => s.Reverse())
            .Reverse()
            .ToList();

        var reversedVertices = reversedSegments
            .Select(s => s.StartPoint)
            .ToList();

        reversedVertices.Add(reversedSegments.Last().EndPoint);

        return new GeoPolycurve
        {
            Segments = reversedSegments,
            Vertices = reversedVertices,
            StartPoint = reversedSegments.First().StartPoint,
            EndPoint = reversedSegments.Last().EndPoint,
            IsClosed = this.IsClosed,
            Length = Length
        };
    }

    public override List<GeoCurves> SplitByParameter(List<double> parameters)
    {
        if (parameters == null || parameters.Count == 0)
            return new List<GeoCurves> { this };

        var sortedParams = parameters
            .Select(p => Math.Max(0, Math.Min(1, p))) // Clamp t
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
                result.Add(piece[1]); // Lấy đoạn giữa localT1 và localT2
            }
            else
            {
                // Đoạn đầu: từ localT1 đến hết segment đầu
                var startSplit = Segments[startIndex].SplitByParameter(new List<double> { localT1 });
                result.Add(startSplit[1]);

                // Các đoạn giữa nguyên vẹn
                for (int i = startIndex + 1; i < endIndex; i++)
                    result.Add(Segments[i]);

                // Đoạn cuối: từ đầu segment cuối tới localT2
                var endSplit = Segments[endIndex].SplitByParameter(new List<double> { localT2 });
                result.Add(endSplit[0]);
            }

            prevT = t;
        }

        // Đoạn cuối cùng nếu còn
        if (prevT < 1.0 - Constants.DefaultTolerance)
        {
            var (startIndex, localT1) = GetSegmentAndLocalT(prevT);
            var endSplit = Segments[startIndex].SplitByParameter(new List<double> { localT1 });
            result.Add(endSplit[1]);
        }

        return result;
    }

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

    #endregion



}