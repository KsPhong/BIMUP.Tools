using Autodesk.AutoCAD.ApplicationServices;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

public class IntersectionResult
{
    #region Properties

    /// <summary>
    /// Loại kết quả giao nhau: Không giao, Giao tại điểm, Giao trùng đoạn, v.v.
    /// </summary>
    public IntersectType Type { get; }

    /// <summary>
    /// Danh sách các điểm giao nhau (nếu có).
    /// </summary>
    public List<Point3d> Points { get; }

    /// <summary>
    /// Danh sách các đoạn cong trùng nhau (nếu có).
    /// </summary>
    public List<GeoCurves> OverlapSegments { get; }

    #endregion

    #region Constructors

    private IntersectionResult(
        IntersectType type,
        List<Point3d> points = null,
        List<GeoCurves> overlapSegments = null)
    {
        Type = type;
        Points = points ?? new List<Point3d>();
        OverlapSegments = overlapSegments ?? new List<GeoCurves>();
    }

    #endregion

    #region Static Factory Methods

    public static IntersectionResult None() =>
        new IntersectionResult(IntersectType.None);

    public static IntersectionResult Intersect(Point3d point) =>
        new IntersectionResult(IntersectType.Intersect, new List<Point3d> { point });

    public static IntersectionResult Intersect(IEnumerable<Point3d> points) =>
        new IntersectionResult(IntersectType.Intersect, points.ToList());

    public static IntersectionResult Overlap(IEnumerable<GeoCurves> segments) =>
        new IntersectionResult(IntersectType.Overlap, null, segments.ToList());

    public static IntersectionResult SameLine(IEnumerable<GeoCurves> segments) =>
        new IntersectionResult(IntersectType.SameLine, null, segments.ToList());

    #endregion

    #region Utilities

    public bool HasIntersection =>
        Type != IntersectType.None &&
        (Points.Count > 0 || OverlapSegments.Count > 0);

    public override string ToString()
    {
        switch (Type)
        {
            case IntersectType.None:
                return "[Intersect] None";

            case IntersectType.Intersect:
                return "[Intersect] Points: " + Points.Count;

            case IntersectType.Overlap:
                return "[Intersect] OverlapSegments: " + OverlapSegments.Count;

            case IntersectType.SameLine:
                return "[Intersect] SameLineSegments: " + OverlapSegments.Count;

            default:
                return "[Intersect] Unknown";
        }
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

public class TestGeo
{
    public static void Draw(Document doc, Transaction tr, Entity[] objects)
    {
        if (doc == null || tr == null || objects == null || objects.Length == 0)
            return;

        var db = doc.Database;
        var bt = (BlockTable)tr.GetObject(db.BlockTableId, OpenMode.ForRead);
        var btr = (BlockTableRecord)tr.GetObject(bt[BlockTableRecord.ModelSpace], OpenMode.ForWrite);

        foreach (var ent in objects)
        {
            btr.AppendEntity(ent);
            tr.AddNewlyCreatedDBObject(ent, true);
        }
    }

}