using Autodesk.AutoCAD.ApplicationServices;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.EditorInput;
using Autodesk.AutoCAD.Geometry;
using Autodesk.AutoCAD.Runtime;
using BIMUP.API;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using Application = Autodesk.AutoCAD.ApplicationServices.Application;


namespace BIMUP.Core
{
    public class TestAPI
    {


        #region Test GeoPlane
        [CommandMethod("TestGeoPlane")]
        public void TestGeoPlane()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            using (var tr = doc.TransactionManager.StartTransaction())
            {
                var plane = new GeoPlane(new Point3d(0, 0, 0), new Vector3d(0, 0, 1));
                var ents = plane.Test();
                TestGeo.Draw(doc, tr, ents);
                tr.Commit();
            }
        }

        [CommandMethod("TestGeoPlaneOn3dPoly")]
        public void TestGeoPlaneOn3dPoly()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var db = doc.Database;
            var ed = doc.Editor;

            // Chọn 3D Polyline
            var res = ed.GetEntity("\nChọn 3D Polyline để kiểm thử GeoPlane:");
            if (res.Status != PromptStatus.OK) return;

            using (var tr = db.TransactionManager.StartTransaction())
            {
                // Mở đối tượng đã chọn
                var poly3d = tr.GetObject(res.ObjectId, OpenMode.ForRead) as Polyline3d;
                if (poly3d == null)
                {
                    ed.WriteMessage("\n❌ Đối tượng không phải là Polyline3d.");
                    return;
                }

                // Tạo GeoPolycurve từ Polyline3d
                var geo = new GeoPolycurve(poly3d);

                // Tính toán và tạo 5 mặt phẳng chia đều
                int count = 5;
                var bt = (BlockTable)tr.GetObject(db.BlockTableId, OpenMode.ForRead);
                var btr = (BlockTableRecord)tr.GetObject(bt[BlockTableRecord.ModelSpace], OpenMode.ForWrite);

                for (int i = 0; i < count; i++)
                {
                    double t = (i + 0.5) / count; // lấy giữa mỗi đoạn chia
                    var plane = geo.PlaneAtParameter(t);
                    var ents = plane.Test();

                    foreach (var ent in ents)
                    {
                        btr.AppendEntity(ent);
                        tr.AddNewlyCreatedDBObject(ent, true);
                    }
                }

                tr.Commit();
                ed.WriteMessage($"\n✅ Đã tạo {count} mặt phẳng chia đều trên tuyến.");
            }
        }
        #endregion

        #region Test CoordinateSystem
        [CommandMethod("TestCoordinate")]
        public void TestCoordinate()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var ed = doc.Editor;
            var db = doc.Database;

            // Chọn đối tượng đường cong (Polyline, Line, Arc, v.v.)
            ObjectIdCollection ids = SelectUtilities.SelectObjects("LINE", "\nChọn đường Line để test hệ tọa độ:");
            if (ids.Count == 0) return;

            using (Transaction tr = db.TransactionManager.StartTransaction())
            {
                var ent = tr.GetObject(ids[0], OpenMode.ForRead) as Line;
                if (ent == null)
                {
                    ed.WriteMessage("\n❌ Không phải là đối tượng Line hợp lệ.");
                    return;
                }

                // Tạo GeoLine từ đối tượng AutoCAD
                GeoLine geo = new GeoLine(ent);

                // Tạo hệ tọa độ tại các tham số t = 0.0, 0.25, 0.5, 0.75, 1.0
                var tValues = new double[] { 0.0, 0.25, 0.5, 0.75, 1.0 };
                var entities = new List<Entity>();

                foreach (var t in tValues)
                {
                    var coordSys = geo.CoordinateSystemAtParameter(t);
                    entities.AddRange(coordSys.Test()); // Test tạo ra 3 đường trục màu
                }

                // Vẽ các hệ trục XYZ ra model
                TestGeo.Draw(doc, tr, entities.ToArray());

                tr.Commit();
            }
        }

        [CommandMethod("TestCoordinateOn3dPolyline")]
        public void TestCoordinateOn3dPolyline()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var db = doc.Database;
            var ed = doc.Editor;

            // Chọn Polyline3d
            var res = ed.GetEntity("\nChọn 3DPolyline để kiểm thử hệ trục:");
            if (res.Status != PromptStatus.OK) return;

            using (var tr = db.TransactionManager.StartTransaction())
            {
                var ent = tr.GetObject(res.ObjectId, OpenMode.ForRead) as Polyline3d;
                if (ent == null)
                {
                    ed.WriteMessage("\n❌ Đối tượng không phải là Polyline3d.");
                    return;
                }

                // Khởi tạo GeoPolycurve
                var geo = new GeoPolycurve(ent);
                var entities = new List<Entity>();

                // Các tham số t thử nghiệm
                var tValues = new double[] { 0.0, 0.25, 0.5, 0.75, 1.0 };

                foreach (var t in tValues)
                {
                    var coordSys = geo.CoordinateSystemAtParameter(t);
                    entities.AddRange(coordSys.Test());
                }

                TestGeo.Draw(doc, tr, entities.ToArray());
                tr.Commit();
                ed.WriteMessage("\n✅ Đã vẽ hệ trục tại các vị trí trên tuyến.");
            }
        }
        #endregion

        #region GeoLine
        [CommandMethod("TestGeoLine")]
        public void TestGeoLine()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var ed = doc.Editor;
            var db = doc.Database;

            // Chọn đối tượng LINE
            ObjectIdCollection ids = SelectUtilities.SelectObjects("LINE", "\n👉 Chọn đường LINE để test GeoLine:");
            if (ids.Count == 0) return;

            using (Transaction tr = db.TransactionManager.StartTransaction())
            {
                var lineEnt = tr.GetObject(ids[0], OpenMode.ForRead) as Line;
                if (lineEnt == null)
                {
                    ed.WriteMessage("\n❌ Đối tượng không phải LINE.");
                    return;
                }

                // Khởi tạo GeoLine từ đối tượng
                var geo = new GeoLine(lineEnt);

                // Danh sách t để kiểm tra
                var tValues = new double[] { 0.0, 0.25, 0.5, 0.75, 1.0 };
                var entities = new List<Entity>();

                // Tạo các hệ tọa độ tại từng tham số
                foreach (var t in tValues)
                {
                    var coord = geo.CoordinateSystemAtParameter(t);
                    entities.AddRange(coord.Test());
                }

                // Vẽ hệ tọa độ
                TestGeo.Draw(doc, tr, entities.ToArray());

                tr.Commit();
                ed.WriteMessage("\n✅ Đã vẽ các hệ tọa độ tại các điểm trên GeoLine.");
            }
        }

        [CommandMethod("Test_GeoLine_PointAtParameter")]
        public void Test_GeoLine_PointAtParameter()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var db = doc.Database;
            var ed = doc.Editor;

            var ids = SelectUtilities.SelectObjects("LINE", "\n👉 Chọn 1 LINE để test PointAtParameter:");
            if (ids.Count == 0) return;

            using (var tr = db.TransactionManager.StartTransaction())
            {
                var line = tr.GetObject(ids[0], OpenMode.ForRead) as Line;
                if (line == null) return;

                var geo = new GeoLine(line);
                var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);

                var tValues = new[] { 0.0, 0.25, 0.5, 0.75, 1.0 };
                foreach (var t in tValues)
                {
                    var pt = geo.PointAtParameter(t);
                    var circle = new Circle(pt, Vector3d.ZAxis, 1.0) { ColorIndex = 2 };
                    btr.AppendEntity(circle);
                    tr.AddNewlyCreatedDBObject(circle, true);
                }

                tr.Commit();
                ed.WriteMessage("\n✅ Đã tạo các vòng tròn tại các tham số t trên GeoLine.");
            }
        }

        [CommandMethod("Test_GeoLine_ParameterAtPoint")]
        public void Test_GeoLine_ParameterAtPoint()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var db = doc.Database;
            var ed = doc.Editor;

            // Chọn LINE
            var lineIds = SelectUtilities.SelectObjects("LINE", "\n👉 Chọn 1 đường LINE:");
            if (lineIds.Count == 0) return;

            // Chọn CIRCLE
            var circleIds = SelectUtilities.SelectObjects("CIRCLE", "\n👉 Chọn 1 đường tròn (CIRCLE):");
            if (circleIds.Count == 0) return;

            using (var tr = db.TransactionManager.StartTransaction())
            {
                var line = tr.GetObject(lineIds[0], OpenMode.ForRead) as Line;
                var circle = tr.GetObject(circleIds[0], OpenMode.ForRead) as Circle;

                if (line == null || circle == null)
                {
                    ed.WriteMessage("\n❌ Không phải LINE hoặc CIRCLE hợp lệ.");
                    return;
                }

                var geo = new GeoLine(line);
                var center = circle.Center;

                // Tính t từ tâm đường tròn
                double t = geo.ParameterAtPoint(center);
                Point3d ptOnLine = geo.PointAtParameter(t);
                ed.WriteMessage($"\n📍 ParameterAtPoint(center) = {t:F4}");

                // Vẽ lại một circle tại điểm đó
                var newCircle = new Circle(ptOnLine, Vector3d.ZAxis, circle.Radius)
                {
                    ColorIndex = 3 // xanh lá
                };

                var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);
                btr.AppendEntity(newCircle);
                tr.AddNewlyCreatedDBObject(newCircle, true);

                tr.Commit();
                ed.WriteMessage("\n✅ Đã vẽ lại đường tròn tại vị trí tương ứng trên GeoLine.");
            }
        }

        [CommandMethod("Test_GeoLine_DistanceTo")]
        public void Test_GeoLine_DistanceTo()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var db = doc.Database;
            var ed = doc.Editor;

            // Chọn LINE
            var lineIds = SelectUtilities.SelectObjects("LINE", "\n👉 Chọn 1 đường LINE:");
            if (lineIds.Count == 0) return;

            // Chọn CIRCLE
            var circleIds = SelectUtilities.SelectObjects("CIRCLE", "\n👉 Chọn 1 đường tròn (CIRCLE):");
            if (circleIds.Count == 0) return;

            using (var tr = db.TransactionManager.StartTransaction())
            {
                var line = tr.GetObject(lineIds[0], OpenMode.ForRead) as Line;
                var circle = tr.GetObject(circleIds[0], OpenMode.ForRead) as Circle;

                if (line == null || circle == null)
                {
                    ed.WriteMessage("\n❌ Không phải LINE hoặc CIRCLE hợp lệ.");
                    return;
                }

                var geo = new GeoLine(line);
                var center = circle.Center;

                // Tính khoảng cách
                double dist = geo.DistanceTo(center);
                ed.WriteMessage($"\n📏 Khoảng cách từ tâm đường tròn đến GeoLine: {dist:F4}");

                // Vẽ lại vòng tròn tại tâm (màu đỏ)
                var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);
                var newCircle = new Circle(center, Vector3d.ZAxis, 1.0)
                {
                    ColorIndex = 1 // đỏ
                };
                btr.AppendEntity(newCircle);
                tr.AddNewlyCreatedDBObject(newCircle, true);

                tr.Commit();
                ed.WriteMessage("\n✅ Đã vẽ lại 1 vòng tròn tại tâm để đánh dấu điểm test.");
            }
        }

        [CommandMethod("Test_GeoLine_CoordinateSystem")]
        public void Test_GeoLine_CoordinateSystem()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var db = doc.Database;
            var ed = doc.Editor;

            var ids = SelectUtilities.SelectObjects("LINE", "\n👉 Chọn LINE để test hệ trục:");
            if (ids.Count == 0) return;

            using (var tr = db.TransactionManager.StartTransaction())
            {
                var line = tr.GetObject(ids[0], OpenMode.ForRead) as Line;
                if (line == null) return;

                var geo = new GeoLine(line);
                var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);

                var tValues = new[] { 0.0, 0.25, 0.5, 0.75, 1.0 };

                foreach (var t in tValues)
                {
                    var cs = geo.CoordinateSystemAtParameter(t);
                    foreach (var ent in cs.Test())
                    {
                        ent.ColorIndex = 2; // hệ thường: xanh lá
                        btr.AppendEntity(ent);
                        tr.AddNewlyCreatedDBObject(ent, true);
                    }

                    var fixZ = geo.CoordinateSystemAtParameterFixZ(t);
                    foreach (var ent in fixZ.Test())
                    {
                        ent.ColorIndex = 6; // hệ fix Z: hồng
                        btr.AppendEntity(ent);
                        tr.AddNewlyCreatedDBObject(ent, true);
                    }
                }

                tr.Commit();
                ed.WriteMessage("\n✅ Đã tạo hệ tọa độ thường và FixZ.");
            }
        }

        [CommandMethod("Test_GeoLine_Split")]
        public void Test_GeoLine_Split()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var db = doc.Database;
            var ed = doc.Editor;

            var ids = SelectUtilities.SelectObjects("LINE", "\n👉 Chọn LINE để test chia đoạn:");
            if (ids.Count == 0) return;

            using (var tr = db.TransactionManager.StartTransaction())
            {
                var line = tr.GetObject(ids[0], OpenMode.ForRead) as Line;
                if (line == null) return;

                var geo = new GeoLine(line);
                var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);

                var parts = geo.SplitByParameter(new List<double> { 0.3, 0.6 });
                int color = 3;

                foreach (var seg in parts)
                {
                    TestGeo.Draw(doc, tr, seg.Test());
                    color++;
                }

                tr.Commit();
                ed.WriteMessage("\n✅ Đã chia và vẽ các đoạn GeoLine.");
            }
        }

        [CommandMethod("Test_GeoLine_PullOntoPlane")]
        public void Test_GeoLine_PullOntoPlane()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var db = doc.Database;
            var ed = doc.Editor;

            // Chọn 1 đường LINE
            var ids = SelectUtilities.SelectObjects("LINE", "\n👉 Chọn LINE để kéo xuống mặt phẳng XOY:");
            if (ids.Count == 0) return;

            using (var tr = db.TransactionManager.StartTransaction())
            {
                var line = tr.GetObject(ids[0], OpenMode.ForRead) as Line;
                if (line == null)
                {
                    ed.WriteMessage("\n❌ Không phải LINE.");
                    return;
                }

                // 1. Khởi tạo GeoLine từ Line
                var geo = new GeoLine(line);

                // 2. Mặt phẳng XOY (Z = 0)
                var plane = new GeoPlane(Point3d.Origin, Vector3d.ZAxis);

                // 3. Kéo Line lên mặt phẳng
                var projected = geo.PullOntoPlane(plane);

                // 4. Vẽ cả Line gốc và Line đã kéo
                var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);

                // Đường gốc (màu xanh dương)
                var original = geo.Test()[0];
                original.ColorIndex = 4;
                btr.AppendEntity(original);
                tr.AddNewlyCreatedDBObject(original, true);

                // Đường đã kéo (màu đỏ)
                var pulled = projected.Test()[0];
                pulled.ColorIndex = 1;
                btr.AppendEntity(pulled);
                tr.AddNewlyCreatedDBObject(pulled, true);

                tr.Commit();
                ed.WriteMessage("\n✅ Đã kéo LINE xuống mặt phẳng XOY và vẽ kết quả.");
            }
        }

        [CommandMethod("Test_GeoPolycurve_PointAtParameter2D")]
        public void Test_GeoPolycurve_PointAtParameter2D()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var db = doc.Database;
            var ed = doc.Editor;

            // Chọn 3D Polyline
            var polyResult = ed.GetEntity("\n👉 Chọn 3D Polyline:");
            if (polyResult.Status != PromptStatus.OK) return;

            // Chọn Circle (tâm là điểm kiểm tra)
            var circleResult = ed.GetEntity("\n👉 Chọn CIRCLE để lấy tâm kiểm tra:");
            if (circleResult.Status != PromptStatus.OK) return;

            using (var tr = db.TransactionManager.StartTransaction())
            {
                var poly3d = tr.GetObject(polyResult.ObjectId, OpenMode.ForRead) as Polyline3d;
                var circle = tr.GetObject(circleResult.ObjectId, OpenMode.ForRead) as Circle;
                if (poly3d == null || circle == null)
                {
                    ed.WriteMessage("\n❌ Đối tượng không hợp lệ.");
                    return;
                }

                var geo = new GeoPolycurve(poly3d);
                var center = circle.Center;

                // Tính t theo mặt phẳng OXY
                double t = geo.ParameterAtPoint2D(center);
                Point3d ptReal = geo.PointAtParameter(t);

                ed.WriteMessage($"\n📍 t (2D) = {t:F4}, điểm trên tuyến = ({ptReal.X:F2}, {ptReal.Y:F2}, {ptReal.Z:F2})");

                var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);

                // Vẽ Circle tại vị trí nội suy
                var resultCircle = new Circle(ptReal, Vector3d.ZAxis, 1.0)
                {
                    ColorIndex = 6 // tím
                };
                btr.AppendEntity(resultCircle);
                tr.AddNewlyCreatedDBObject(resultCircle, true);

                tr.Commit();
                ed.WriteMessage("\n✅ Đã vẽ CIRCLE tại điểm gần nhất theo chiếu OXY.");
            }
        }

        [CommandMethod("Test_GeoLine_IntersectWith")]
        public void Test_GeoLine_IntersectWith()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var db = doc.Database;
            var ed = doc.Editor;

            // Chọn 2 LINE
            var ids = SelectUtilities.SelectObjects("LINE", "\n👉 Chọn 2 LINE để test giao nhau:");
            if (ids.Count < 2)
            {
                ed.WriteMessage("\n❌ Cần chọn đúng 2 LINE.");
                return;
            }

            using (var tr = db.TransactionManager.StartTransaction())
            {
                var line1 = tr.GetObject(ids[0], OpenMode.ForRead) as Line;
                var line2 = tr.GetObject(ids[1], OpenMode.ForRead) as Line;

                if (line1 == null || line2 == null)
                {
                    ed.WriteMessage("\n❌ Không hợp lệ.");
                    return;
                }

                // Chuyển thành GeoLine
                var geo1 = new GeoLine(line1);
                var geo2 = new GeoLine(line2);

                // Tính giao
                var result = geo1.IntersectWith(geo2);
                ed.WriteMessage($"\n🔎 Giao kết quả: {result}");

                var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);

                switch (result.Type)
                {
                    case IntersectionResult.IntersectType.Intersect:
                        // Vẽ đường tròn tại điểm giao
                        foreach (var pt in result.Points)
                        {
                            var circ = new Circle(pt, Vector3d.ZAxis, 1.0) { ColorIndex = 1 }; // đỏ
                            btr.AppendEntity(circ);
                            tr.AddNewlyCreatedDBObject(circ, true);
                        }
                        break;

                    case IntersectionResult.IntersectType.Overlap:
                    case IntersectionResult.IntersectType.SameLine:
                        // Vẽ các đoạn trùng
                        foreach (var seg in result.OverlapSegments)
                        {
                            var lineEnt = seg.Test()[0];
                            lineEnt.ColorIndex = 3; // xanh lá
                            btr.AppendEntity(lineEnt);
                            tr.AddNewlyCreatedDBObject(lineEnt, true);
                        }
                        break;

                    default:
                        ed.WriteMessage("\n❌ Không có giao nhau.");
                        break;
                }

                tr.Commit();
                ed.WriteMessage("\n✅ Đã vẽ kết quả giao nhau.");
            }
        }

        [CommandMethod("Test_GeoLine_IntersectWithArc")]
        public void Test_GeoLine_IntersectWithArc()
        {
            Document doc = Application.DocumentManager.MdiActiveDocument;
            Editor ed = doc.Editor;
            Database db = doc.Database;

            ObjectIdCollection lineIds = SelectUtilities.SelectObjects("LINE", "\n👉 Chọn một đường Line: ");
            if (lineIds.Count == 0)
            {
                ed.WriteMessage("\n⚠️ Chưa chọn đối tượng nào.");
                return;
            }

            ObjectIdCollection arcIds = SelectUtilities.SelectObjects("ARC", "\n👉 Chọn một cung tròn: ");
            if (arcIds.Count == 0)
            {
                ed.WriteMessage("\n⚠️ Chưa chọn đối tượng nào.");
                return;
            }

            ObjectId lineId = lineIds[0];
            ObjectId arcId = arcIds[0];

            using (Transaction tr = db.TransactionManager.StartTransaction())
            {
                try
                {
                    var acadLine = tr.GetObject(lineId, OpenMode.ForRead) as Line;
                    var acadArc = tr.GetObject(arcId, OpenMode.ForRead) as Arc;

                    if (acadLine == null || acadArc == null)
                    {
                        ed.WriteMessage("\n❌ Lỗi: Một trong các đối tượng được chọn không hợp lệ.");
                        return;
                    }

                    // Log thông tin Line
                    ed.WriteMessage($"\n🔹 LINE: Start = {acadLine.StartPoint}, End = {acadLine.EndPoint}");
                    ed.WriteMessage($"\n🔹 Direction: {acadLine.Delta}");

                    // Log thông tin Arc
                    ed.WriteMessage($"\n🔸 ARC: Center = {acadArc.Center}, Radius = {acadArc.Radius:0.####}");
                    ed.WriteMessage($"\n🔸 Start = {acadArc.StartPoint}, End = {acadArc.EndPoint}");
                    ed.WriteMessage($"\n🔸 Normal = {acadArc.Normal}");

                    var geoLine = new GeoLine(acadLine);
                    var geoArc = new GeoArc(acadArc);

                    // Kiểm tra giao điểm
                    ed.WriteMessage("\n🔍 Tiến hành kiểm tra giao nhau...");
                    IntersectionResult intersection = geoLine.IntersectWith(geoArc);

                    if (intersection.Type == IntersectionResult.IntersectType.Intersect)
                    {
                        ed.WriteMessage($"\n✅ Tìm thấy {intersection.Points.Count} giao điểm!");

                        var blockTable = tr.GetObject(db.BlockTableId, OpenMode.ForRead) as BlockTable;
                        var modelSpace = tr.GetObject(blockTable[BlockTableRecord.ModelSpace], OpenMode.ForWrite) as BlockTableRecord;

                        int i = 1;
                        foreach (var pt in intersection.Points)
                        {
                            ed.WriteMessage($"\n   🔸 Giao điểm {i++}: {pt}");

                            var circle = new Circle()
                            {
                                Center = pt,
                                Radius = 5.0,
                                Normal = acadArc.Normal,
                                ColorIndex = 2
                            };

                            modelSpace.AppendEntity(circle);
                            tr.AddNewlyCreatedDBObject(circle, true);
                        }

                        ed.WriteMessage("\n🎯 Đã vẽ xong các điểm giao.");
                    }
                    else
                    {
                        ed.WriteMessage("\n❌ Không tìm thấy giao điểm nào.");
                    }

                    tr.Commit();
                }
                catch (System.Exception ex)
                {
                    ed.WriteMessage($"\n💥 Lỗi: {ex.Message}");
                    tr.Abort();
                }
            }
        }

        #endregion

        #region GeoArc


        [CommandMethod("Test_GeoArc_Info")]
        public void Test_GeoArc_Info()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var db = doc.Database;
            var ed = doc.Editor;

            var result = ed.GetEntity("\n👉 Chọn một đối tượng ARC:");
            if (result.Status != PromptStatus.OK) return;

            using (var tr = db.TransactionManager.StartTransaction())
            {
                var arc = tr.GetObject(result.ObjectId, OpenMode.ForRead) as Arc;
                if (arc == null)
                {
                    ed.WriteMessage("\n❌ Đối tượng không phải ARC.");
                    return;
                }

                var ptStart = arc.StartPoint;
                var ptEnd = arc.EndPoint;
                var ptMid = arc.GetPointAtParameter((arc.StartParam + arc.EndParam) / 2.0);

                GeoArc geo;
                try
                {
                    geo = new GeoArc(ptStart, ptMid, ptEnd);
                }
                catch (System.Exception ex)
                {
                    ed.WriteMessage($"\n❌ Không thể dựng GeoArc từ 3 điểm: {ex.Message}");
                    return;
                }

                // In thông tin dạng bảng Geometry
                ed.WriteMessage("\n===== GeoArc Geometry Info =====");
                ed.WriteMessage($"\nStart X\t{geo.StartPoint.X:0.####}");
                ed.WriteMessage($"\nStart Y\t{geo.StartPoint.Y:0.####}");
                ed.WriteMessage($"\nStart Z\t{geo.StartPoint.Z:0.####}");

                ed.WriteMessage($"\nCenter X\t{geo.CenterPoint.X:0.####}");
                ed.WriteMessage($"\nCenter Y\t{geo.CenterPoint.Y:0.####}");
                ed.WriteMessage($"\nCenter Z\t{geo.CenterPoint.Z:0.####}");

                ed.WriteMessage($"\nEnd X\t{geo.EndPoint.X:0.####}");
                ed.WriteMessage($"\nEnd Y\t{geo.EndPoint.Y:0.####}");
                ed.WriteMessage($"\nEnd Z\t{geo.EndPoint.Z:0.####}");

                ed.WriteMessage($"\nRadius\t{geo.Radius:0.####}");

                var xAxis = (geo.StartPoint - geo.CenterPoint).GetNormal();
                var vStart = (geo.StartPoint - geo.CenterPoint).GetNormal();
                var vEnd = (geo.EndPoint - geo.CenterPoint).GetNormal();
                double startAngle = 0;
                double endAngle = xAxis.GetAngleTo(vEnd, geo.Normal);
                double totalAngle = geo.Angle;
                double arcLength = geo.Radius * geo.Angle;
                double area = 0.5 * geo.Radius * geo.Radius * geo.Angle;

                ed.WriteMessage($"\nStart angle\t{startAngle * 180 / Math.PI:0.####}");
                ed.WriteMessage($"\nEnd angle\t{endAngle * 180 / Math.PI:0.####}");
                ed.WriteMessage($"\nTotal angle\t{totalAngle * 180 / Math.PI:0.####}");
                ed.WriteMessage($"\nArc length\t{arcLength:0.####}");
                ed.WriteMessage($"\nArea\t{area:0.####}");

                ed.WriteMessage($"\nNormal X\t{geo.Normal.X:0.####}");
                ed.WriteMessage($"\nNormal Y\t{geo.Normal.Y:0.####}");
                ed.WriteMessage($"\nNormal Z\t{geo.Normal.Z:0.####}");

                ed.WriteMessage("\n===============================");

                tr.Commit();
            }
        }

        [CommandMethod("Test_GeoArc_RebuildFrom3Points")]
        public void Test_GeoArc_RebuildFrom3Points()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var db = doc.Database;
            var ed = doc.Editor;

            // B1: Chọn một đối tượng ARC
            var result = ed.GetEntity("\n👉 Chọn một đối tượng ARC:");
            if (result.Status != PromptStatus.OK) return;

            using (var tr = db.TransactionManager.StartTransaction())
            {
                var arc = tr.GetObject(result.ObjectId, OpenMode.ForRead) as Arc;
                if (arc == null)
                {
                    ed.WriteMessage("\n❌ Đối tượng không phải ARC.");
                    return;
                }

                // B2: Lấy 3 điểm đặc trưng
                var ptStart = arc.StartPoint;
                var ptEnd = arc.EndPoint;
                var ptMid = arc.GetPointAtParameter((arc.StartParam + arc.EndParam) / 2.0);

                // B3: Dựng GeoArc từ 3 điểm đó
                GeoArc geo;
                try
                {
                    geo = new GeoArc(ptStart, ptMid, ptEnd);
                }
                catch (System.Exception ex)
                {
                    ed.WriteMessage($"\n❌ Không thể dựng GeoArc từ 3 điểm: {ex.Message}");
                    return;
                }

                // B4: Dựng Arc mới từ GeoArc bằng hệ tọa độ mặt phẳng chuẩn
                var xAxis = (geo.StartPoint - geo.CenterPoint).GetNormal();
                var yAxis = geo.Normal.CrossProduct(xAxis).GetNormal();

                var vStart = (geo.StartPoint - geo.CenterPoint).GetNormal();
                var vEnd = (geo.EndPoint - geo.CenterPoint).GetNormal();
                var xRef = xAxis;

                double startAngle = xRef.GetAngleTo(vStart, geo.Normal);
                double endAngle = xRef.GetAngleTo(vEnd, geo.Normal);

                var rebuiltArc = new Arc(
                    geo.CenterPoint,
                    geo.Normal,
                    geo.Radius,
                    startAngle,
                    endAngle
                )
                {
                    ColorIndex = 5
                };

                // B5: Vẽ kết quả
                var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);
                btr.AppendEntity(rebuiltArc); tr.AddNewlyCreatedDBObject(rebuiltArc, true);

                // Vẽ lại các điểm để đối chiếu
                var c1 = new Circle(ptStart, geo.Normal, 0.4) { ColorIndex = 1 }; // đỏ
                var c2 = new Circle(ptMid, geo.Normal, 0.4) { ColorIndex = 3 };   // xanh dương
                var c3 = new Circle(ptEnd, geo.Normal, 0.4) { ColorIndex = 2 };   // xanh lá

                btr.AppendEntity(c1); tr.AddNewlyCreatedDBObject(c1, true);
                btr.AppendEntity(c2); tr.AddNewlyCreatedDBObject(c2, true);
                btr.AppendEntity(c3); tr.AddNewlyCreatedDBObject(c3, true);

                tr.Commit();

                ed.WriteMessage("\n✅ Đã dựng lại GeoArc từ 3 điểm của ARC gốc và vẽ lại.");
            }
        }

        [CommandMethod("Test_GeoArc_3PointsWithCircle")]
        public void Test_GeoArc_3PointsWithCircle()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var db = doc.Database;
            var ed = doc.Editor;

            // Chọn một đối tượng ARC
            var result = ed.GetEntity("\n👉 Chọn một đối tượng ARC để kiểm tra 3 điểm đặc trưng:");
            if (result.Status != PromptStatus.OK) return;

            using (var tr = db.TransactionManager.StartTransaction())
            {
                var arc = tr.GetObject(result.ObjectId, OpenMode.ForRead) as Arc;
                if (arc == null)
                {
                    ed.WriteMessage("\n❌ Đối tượng không phải ARC.");
                    return;
                }

                // Tạo GeoArc từ Arc
                var geo = new GeoArc(arc);

                // Tạo lại Arc từ GeoArc để xác nhận hình học đúng
                var newArc = new Arc(
                    geo.CenterPoint,
                    geo.Normal,
                    geo.Radius,
                    geo.StartAngle,
                    geo.EndAngle
                )
                {
                    ColorIndex = 5 // Tím nhạt - phân biệt với Arc gốc
                };

                // Vẽ 3 điểm đặc trưng bằng Circle
                var cStart = new Circle(geo.StartPoint, geo.Normal, 0.4) { ColorIndex = 1 }; // Đỏ
                var cMid = new Circle(geo.MidPoint, geo.Normal, 0.4) { ColorIndex = 3 };   // Xanh dương
                var cEnd = new Circle(geo.EndPoint, geo.Normal, 0.4) { ColorIndex = 2 };   // Xanh lá

                // Ghi vào bản vẽ
                var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);
                btr.AppendEntity(newArc); tr.AddNewlyCreatedDBObject(newArc, true);
                btr.AppendEntity(cStart); tr.AddNewlyCreatedDBObject(cStart, true);
                btr.AppendEntity(cMid); tr.AddNewlyCreatedDBObject(cMid, true);
                btr.AppendEntity(cEnd); tr.AddNewlyCreatedDBObject(cEnd, true);

                tr.Commit();

                ed.WriteMessage("\n✅ Đã vẽ cung mới và 3 điểm đặc trưng của GeoArc.");
                ed.WriteMessage($"\n🔴 Start: ({geo.StartPoint.X:F2}, {geo.StartPoint.Y:F2}, {geo.StartPoint.Z:F2})");
                ed.WriteMessage($"\n🔵 Mid:   ({geo.MidPoint.X:F2}, {geo.MidPoint.Y:F2}, {geo.MidPoint.Z:F2})");
                ed.WriteMessage($"\n🟢 End:   ({geo.EndPoint.X:F2}, {geo.EndPoint.Y:F2}, {geo.EndPoint.Z:F2})");
            }
        }

        [CommandMethod("Test_GeoArc_ConstructorFromArc")]
        public void Test_GeoArc_ConstructorFromArc()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var db = doc.Database;
            var ed = doc.Editor;

            // Bước 1: Yêu cầu người dùng chọn 1 Arc
            var result = ed.GetEntity("\n👉 Chọn một đối tượng ARC:");
            if (result.Status != PromptStatus.OK) return;

            using (var tr = db.TransactionManager.StartTransaction())
            {
                var arc = tr.GetObject(result.ObjectId, OpenMode.ForRead) as Arc;
                if (arc == null)
                {
                    ed.WriteMessage("\n❌ Đối tượng không phải ARC.");
                    return;
                }

                // Bước 2: Tạo GeoArc từ Arc
                GeoArc geo = new GeoArc(arc);

                // Bước 3: Tạo Arc mới từ GeoArc để vẽ kiểm tra
                var rebuilt = new Arc(
                    geo.CenterPoint,
                    geo.Normal,
                    geo.Radius,
                    geo.StartAngle,
                    geo.EndAngle
                )
                {
                    ColorIndex = 2 // Xanh lá: để phân biệt với Arc gốc
                };

                // Bước 4: Ghi ra bản vẽ
                var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);
                btr.AppendEntity(rebuilt);
                tr.AddNewlyCreatedDBObject(rebuilt, true);

                // Bước 5: Vẽ 3 điểm đặc trưng để xác minh
                var p1 = new DBPoint(geo.StartPoint) { ColorIndex = 1 }; // đỏ
                var p2 = new DBPoint(geo.MidPoint) { ColorIndex = 3 };   // xanh dương
                var p3 = new DBPoint(geo.EndPoint) { ColorIndex = 1 };   // đỏ
                btr.AppendEntity(p1); tr.AddNewlyCreatedDBObject(p1, true);
                btr.AppendEntity(p2); tr.AddNewlyCreatedDBObject(p2, true);
                btr.AppendEntity(p3); tr.AddNewlyCreatedDBObject(p3, true);

                tr.Commit();
                ed.WriteMessage("\n✅ Đã test GeoArc(Arc) constructor và vẽ kết quả.");
            }
        }

        [CommandMethod("Test_GeoArc_PointAtParameter")]
        public void Test_GeoArc_PointAtParameter()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;       
            if (doc == null) return;
            var db = doc.Database;
            var ed = doc.Editor;

            var ids = SelectUtilities.SelectObjects("ARC", "\n👉 Chọn 1 ARC để test PointAtParameter:");
            if (ids.Count == 0) return;

            using (var tr = db.TransactionManager.StartTransaction())
            {
                var arc = tr.GetObject(ids[0], OpenMode.ForRead) as Arc;
                if (arc == null) return;

                var geo = new GeoArc(arc);
                var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);

                var tValues = new[] { 0.0, 0.25, 0.5, 0.75, 1.0 };
                foreach (var t in tValues)
                {
                    var pt = geo.PointAtParameter(t);
                    var circle = new Circle(pt, Vector3d.ZAxis, 1.0) { ColorIndex = 2 };
                    btr.AppendEntity(circle);
                    tr.AddNewlyCreatedDBObject(circle, true);
                }

                tr.Commit();
                ed.WriteMessage("\n✅ Đã tạo các vòng tròn tại các tham số t trên GeoArc.");
            }
        }

        [CommandMethod("Test_GeoArc_ParameterAtPoint")]
        public void Test_GeoArc_ParameterAtPoint()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var db = doc.Database;
            var ed = doc.Editor;

            var arcIds = SelectUtilities.SelectObjects("ARC", "\n👉 Chọn 1 ARC:");
            var circleIds = SelectUtilities.SelectObjects("CIRCLE", "\n👉 Chọn 1 CIRCLE:");

            if (arcIds.Count == 0 || circleIds.Count == 0) return;

            using (var tr = db.TransactionManager.StartTransaction())
            {
                var arc = tr.GetObject(arcIds[0], OpenMode.ForRead) as Arc;
                var circle = tr.GetObject(circleIds[0], OpenMode.ForRead) as Circle;
                if (arc == null || circle == null)
                {
                    ed.WriteMessage("\n❌ Không hợp lệ.");
                    return;
                }

                var geo = new GeoArc(arc);
                var t = geo.ParameterAtPoint(circle.Center);
                var pt = geo.PointAtParameter(t);

                ed.WriteMessage($"\n📍 ParameterAtPoint(center) = {t:F4}");

                var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);
                var resultCircle = new Circle(pt, Vector3d.ZAxis, circle.Radius) { ColorIndex = 3 };
                btr.AppendEntity(resultCircle);
                tr.AddNewlyCreatedDBObject(resultCircle, true);

                tr.Commit();
                ed.WriteMessage("\n✅ Đã vẽ lại đường tròn tại vị trí tương ứng trên GeoArc.");
            }
        }

        [CommandMethod("Test_GeoArc_DistanceTo")]
        public void Test_GeoArc_DistanceTo()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var db = doc.Database;
            var ed = doc.Editor;

            var arcIds = SelectUtilities.SelectObjects("ARC", "\n👉 Chọn ARC:");
            var circleIds = SelectUtilities.SelectObjects("CIRCLE", "\n👉 Chọn CIRCLE:");

            if (arcIds.Count == 0 || circleIds.Count == 0) return;

            using (var tr = db.TransactionManager.StartTransaction())
            {
                var arc = tr.GetObject(arcIds[0], OpenMode.ForRead) as Arc;
                var circle = tr.GetObject(circleIds[0], OpenMode.ForRead) as Circle;

                if (arc == null || circle == null)
                {
                    ed.WriteMessage("\n❌ Không hợp lệ.");
                    return;
                }

                var geo = new GeoArc(arc);
                double dist = geo.DistanceTo(circle.Center);
                ed.WriteMessage($"\n📏 Khoảng cách từ tâm CIRCLE đến GeoArc: {dist:F4}");

                var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);
                var testCircle = new Circle(circle.Center, Vector3d.ZAxis, 1.0) { ColorIndex = 1 };
                btr.AppendEntity(testCircle);
                tr.AddNewlyCreatedDBObject(testCircle, true);

                tr.Commit();
                ed.WriteMessage("\n✅ Đã vẽ lại điểm test trên GeoArc.");
            }
        }

        [CommandMethod("Test_GeoArc_CoordinateSystem")]
        public void Test_GeoArc_CoordinateSystem()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var db = doc.Database;
            var ed = doc.Editor;

            var ids = SelectUtilities.SelectObjects("ARC", "\n👉 Chọn ARC để test hệ trục:");
            if (ids.Count == 0) return;

            using (var tr = db.TransactionManager.StartTransaction())
            {
                var arc = tr.GetObject(ids[0], OpenMode.ForRead) as Arc;
                if (arc == null) return;

                var geo = new GeoArc(arc);
                var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);

                var tValues = new[] { 0.0, 0.25, 0.5, 0.75, 1.0 };
                foreach (var t in tValues)
                {
                    var cs = geo.CoordinateSystemAtParameter(t);
                    foreach (var ent in cs.Test())
                    {
                        ent.ColorIndex = 2;
                        btr.AppendEntity(ent);
                        tr.AddNewlyCreatedDBObject(ent, true);
                    }

                    var fixZ = geo.CoordinateSystemAtParameterFixZ(t);
                    foreach (var ent in fixZ.Test())
                    {
                        ent.ColorIndex = 6;
                        btr.AppendEntity(ent);
                        tr.AddNewlyCreatedDBObject(ent, true);
                    }
                }

                tr.Commit();
                ed.WriteMessage("\n✅ Đã vẽ hệ trục thường và fixZ trên GeoArc.");
            }
        }

        [CommandMethod("Test_GeoArc_From3Points")]
        public void Test_GeoArc_From3Points()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var db = doc.Database;
            var ed = doc.Editor;

            var opts = new PromptPointOptions("\n👉 Chọn điểm **bắt đầu**:");
            var res1 = ed.GetPoint(opts);
            if (res1.Status != PromptStatus.OK) return;

            opts.Message = "\n👉 Chọn điểm **ở giữa** (trên cung tròn):";
            var res2 = ed.GetPoint(opts);
            if (res2.Status != PromptStatus.OK) return;

            opts.Message = "\n👉 Chọn điểm **kết thúc**:";
            var res3 = ed.GetPoint(opts);
            if (res3.Status != PromptStatus.OK) return;

            Point3d pt1 = res1.Value;
            Point3d pt2 = res2.Value;
            Point3d pt3 = res3.Value;

            using (var tr = db.TransactionManager.StartTransaction())
            {
                var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);

                try
                {
                    var geo = new GeoArc(pt1, pt2, pt3);
                    var ents = geo.Test(); // tạo Arc AutoCAD

                    foreach (var ent in ents)
                    {
                        btr.AppendEntity(ent);
                        tr.AddNewlyCreatedDBObject(ent, true);
                    }

                    // Vẽ các điểm được chọn
                    var c1 = new Circle(pt1, Vector3d.ZAxis, 0.4) { ColorIndex = 2 }; // Xanh lá
                    var c2 = new Circle(pt2, Vector3d.ZAxis, 0.4) { ColorIndex = 3 }; // Xanh nhạt
                    var c3 = new Circle(pt3, Vector3d.ZAxis, 0.4) { ColorIndex = 1 }; // Đỏ

                    btr.AppendEntity(c1); tr.AddNewlyCreatedDBObject(c1, true);
                    btr.AppendEntity(c2); tr.AddNewlyCreatedDBObject(c2, true);
                    btr.AppendEntity(c3); tr.AddNewlyCreatedDBObject(c3, true);

                    ed.WriteMessage("\n✅ Đã tạo GeoArc từ 3 điểm.");
                }
                catch (System.Exception ex)
                {
                    ed.WriteMessage($"\n❌ Không tạo được GeoArc: {ex.Message}");
                }

                tr.Commit();
            }
        }

        [CommandMethod("Test_GeoArc_PullOntoPlane")]
        public void Test_GeoArc_PullOntoPlane()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var db = doc.Database;
            var ed = doc.Editor;

            var ids = SelectUtilities.SelectObjects("ARC", "\n👉 Chọn ARC để kéo xuống mặt phẳng XOY:");
            if (ids.Count == 0) return;

            using (var tr = db.TransactionManager.StartTransaction())
            {
                var arc = tr.GetObject(ids[0], OpenMode.ForRead) as Arc;
                if (arc == null)
                {
                    ed.WriteMessage("\n❌ Không phải ARC.");
                    return;
                }

                var geo = new GeoArc(arc);
                var plane = new GeoPlane(Point3d.Origin, Vector3d.ZAxis);
                var projected = geo.PullOntoPlane(plane);

                var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);
                var original = geo.Test()[0]; original.ColorIndex = 4;
                var pulled = projected.Test()[0]; pulled.ColorIndex = 1;

                btr.AppendEntity(original); tr.AddNewlyCreatedDBObject(original, true);
                btr.AppendEntity(pulled); tr.AddNewlyCreatedDBObject(pulled, true);

                tr.Commit();
                ed.WriteMessage("\n✅ Đã kéo GeoArc xuống mặt phẳng XOY và vẽ kết quả.");
            }
        }

        [CommandMethod("Test_GeoArc_IntersectWith_Arc")]
        public void Test_GeoArc_IntersectWith_Arc()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var db = doc.Database;
            var ed = doc.Editor;

            // Chọn 2 ARC
            var ids = SelectUtilities.SelectObjects("ARC", "\n👉 Chọn 2 ARC để test giao nhau:");
            if (ids.Count < 2)
            {
                ed.WriteMessage("\n❌ Cần chọn đúng 2 ARC.");
                return;
            }

            using (var tr = db.TransactionManager.StartTransaction())
            {
                var arc1 = tr.GetObject(ids[0], OpenMode.ForRead) as Arc;
                var arc2 = tr.GetObject(ids[1], OpenMode.ForRead) as Arc;

                if (arc1 == null || arc2 == null)
                {
                    ed.WriteMessage("\n❌ Không hợp lệ.");
                    return;
                }

                var geo1 = new GeoArc(arc1.StartPoint, arc1.GetPointAtParameter(0.5), arc1.EndPoint);
                var geo2 = new GeoArc(arc2.StartPoint, arc2.GetPointAtParameter(0.5), arc2.EndPoint);

                var result = geo1.IntersectWith(geo2);
                ed.WriteMessage($"\n🔎 Giao kết quả: {result}");

                var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);

                switch (result.Type)
                {
                    case IntersectionResult.IntersectType.Intersect:
                        foreach (var pt in result.Points)
                        {
                            var circ = new Circle(pt, Vector3d.ZAxis, 1.0) { ColorIndex = 1 }; // đỏ
                            btr.AppendEntity(circ);
                            tr.AddNewlyCreatedDBObject(circ, true);
                        }
                        break;

                    case IntersectionResult.IntersectType.Overlap:
                    case IntersectionResult.IntersectType.SameLine:
                        foreach (var seg in result.OverlapSegments)
                        {
                            var ent = seg.Test()[0]; ent.ColorIndex = 3;
                            btr.AppendEntity(ent);
                            tr.AddNewlyCreatedDBObject(ent, true);
                        }
                        break;

                    default:
                        ed.WriteMessage("\n❌ Không có giao nhau.");
                        break;
                }

                tr.Commit();
                ed.WriteMessage("\n✅ Đã vẽ kết quả giao nhau giữa 2 ARC.");
            }
        }

        [CommandMethod("Test_GeoArc_IntersectWith_Line")]
        public void Test_GeoArc_IntersectWith_Line()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var db = doc.Database;
            var ed = doc.Editor;

            // Chọn 1 ARC và 1 LINE
            var arcIds = SelectUtilities.SelectObjects("ARC", "\n👉 Chọn 1 ARC:");
            var lineIds = SelectUtilities.SelectObjects("LINE", "\n👉 Chọn 1 LINE:");

            if (arcIds.Count == 0 || lineIds.Count == 0)
            {
                ed.WriteMessage("\n❌ Cần chọn 1 ARC và 1 LINE.");
                return;
            }

            using (var tr = db.TransactionManager.StartTransaction())
            {
                var arc = tr.GetObject(arcIds[0], OpenMode.ForRead) as Arc;
                var line = tr.GetObject(lineIds[0], OpenMode.ForRead) as Line;

                if (arc == null || line == null)
                {
                    ed.WriteMessage("\n❌ Không hợp lệ.");
                    return;
                }

                var geoArc = new GeoArc(arc);
                var geoLine = new GeoLine(line);

                var result = geoArc.IntersectWith(geoLine);
                ed.WriteMessage($"\n🔎 Giao kết quả: {result}");

                var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);

                switch (result.Type)
                {
                    case IntersectionResult.IntersectType.Intersect:
                        foreach (var pt in result.Points)
                        {
                            var circ = new Circle(pt, Vector3d.ZAxis, 1.0) { ColorIndex = 1 }; // đỏ
                            btr.AppendEntity(circ);
                            tr.AddNewlyCreatedDBObject(circ, true);
                        }
                        break;

                    case IntersectionResult.IntersectType.Overlap:
                    case IntersectionResult.IntersectType.SameLine:
                        foreach (var seg in result.OverlapSegments)
                        {
                            var ent = seg.Test()[0]; ent.ColorIndex = 3;
                            btr.AppendEntity(ent);
                            tr.AddNewlyCreatedDBObject(ent, true);
                        }
                        break;

                    default:
                        ed.WriteMessage("\n❌ Không có giao nhau.");
                        break;
                }

                tr.Commit();
                ed.WriteMessage("\n✅ Đã vẽ kết quả giao nhau ARC - LINE.");
            }
        }
        [CommandMethod("Test_GeoArc_Reconstruct")]
        public void Test_GeoArc_Reconstruct()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var db = doc.Database;
            var ed = doc.Editor;

            // Chọn 1 đối tượng ARC
            var result = ed.GetEntity("\n👉 Chọn một ARC để kiểm thử GeoArc:");
            if (result.Status != PromptStatus.OK) return;

            using (var tr = db.TransactionManager.StartTransaction())
            {
                var arc = tr.GetObject(result.ObjectId, OpenMode.ForRead) as Arc;
                if (arc == null)
                {
                    ed.WriteMessage("\n❌ Đối tượng không phải ARC.");
                    return;
                }

                // Lấy 3 điểm đặc trưng
                Point3d start = arc.StartPoint;
                Point3d mid = arc.GetPointAtParameter(0.5);
                Point3d end = arc.EndPoint;

                // Dựng GeoArc từ 3 điểm
                var geo = new GeoArc(start, mid, end);

                // Tạo Arc từ dữ liệu GeoArc
                var rebuilt = new Arc(
                    geo.CenterPoint,
                    geo.Normal,
                    geo.Radius,
                    geo.StartAngle,
                    geo.EndAngle
                )
                {
                    ColorIndex = 2 // Xanh lá: để phân biệt
                };

                // Ghi vào bản vẽ
                var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);
                btr.AppendEntity(rebuilt);
                tr.AddNewlyCreatedDBObject(rebuilt, true);

                tr.Commit();
            }
        }

        [CommandMethod("Test_GeoArc_SplitByPoints")]
        public void Test_GeoArc_SplitByPoints()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            if (doc == null) return;
            var db = doc.Database;
            var ed = doc.Editor;

            // Bước 1: Yêu cầu người dùng chọn một đối tượng Arc có sẵn
            var pEntOptions = new PromptEntityOptions("\n🟡 Chọn cung tròn (Arc) để cắt:");
            pEntOptions.SetRejectMessage("\nĐối tượng phải là một Arc.");
            pEntOptions.AddAllowedClass(typeof(Arc), true);
            var pEntResult = ed.GetEntity(pEntOptions);
            if (pEntResult.Status != PromptStatus.OK) return;

            // Bước 2: Yêu cầu người dùng chọn nhiều điểm để cắt
            var splitPoints = new List<Point3d>();
            while (true)
            {
                var pPtOpts = new PromptPointOptions("\n🟡 Chọn điểm để cắt (hoặc nhấn Enter/ESC để kết thúc):");
                pPtOpts.AllowNone = true;
                var pPtResult = ed.GetPoint(pPtOpts);

                if (pPtResult.Status == PromptStatus.Cancel) return;
                if (pPtResult.Status == PromptStatus.None) break; // Kết thúc chọn điểm khi nhấn Enter

                splitPoints.Add(pPtResult.Value);
            }

            if (splitPoints.Count == 0)
            {
                ed.WriteMessage("\nBạn chưa chọn điểm nào để cắt.");
                return;
            }

            // Bước 3: Thực hiện cắt và tạo đối tượng mới
            try
            {
                using (var tr = db.TransactionManager.StartTransaction())
                {
                    // Lấy đối tượng Arc gốc
                    var sourceArc = tr.GetObject(pEntResult.ObjectId, OpenMode.ForRead) as Arc;
                    if (sourceArc == null)
                    {
                        ed.WriteMessage("\nKhông thể đọc đối tượng Arc.");
                        return;
                    }

                    // Dựng GeoArc từ Arc gốc
                    var geoArc = new GeoArc(sourceArc);

                    // Gọi hàm cắt theo danh sách điểm
                    List<GeoCurves> arcSegments = geoArc.SplitByPoints(splitPoints);

                    // Chuẩn bị để tạo các đối tượng hình học mới
                    var colors = new short[] { 1, 2, 3, 4, 5, 6 }; // Mảng màu: Đỏ, Vàng, Lục, Lam...
                    int colorIndex = 0;

                    var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);

                    // Duyệt qua các đoạn cung tròn con và tạo entity cho chúng
                    foreach (var segment in arcSegments)
                    {
                        if (segment is GeoArc smallArc)
                        {
                            // Dùng hàm .Test() để tạo đối tượng Arc từ GeoArc
                            Entity[] segmentEntities = smallArc.Test();
                            if (segmentEntities.Length > 0)
                            {
                                Entity entity = segmentEntities[0];
                                entity.ColorIndex = colors[colorIndex % colors.Length]; // Gán màu theo vòng lặp

                                btr.AppendEntity(entity);
                                tr.AddNewlyCreatedDBObject(entity, true);
                                colorIndex++;
                            }
                        }
                    }

                    // Xóa cung tròn gốc đi để chỉ hiển thị kết quả
                    sourceArc.UpgradeOpen();
                    sourceArc.Erase();

                    tr.Commit();
                }
            }
            catch (System.Exception ex)
            {
                ed.WriteMessage($"\n❌ Lỗi: {ex.Message}");
            }
        }

        #endregion

        #region Polycurve
        [CommandMethod("BIMUP_GeoPolyCurve_CoordinateSystemAtParameter")]
        public void TestCoordinateSystemOriginOn3dPoly()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            if (doc == null) return;
            var db = doc.Database;
            var ed = doc.Editor;

            // Bước 1: Cho người dùng chọn 1 3DPolyline
            var entOptions = new PromptEntityOptions("\n👉 Chọn một đối tượng 3D Polyline: ");
            entOptions.SetRejectMessage("\nĐối tượng được chọn phải là 3D Polyline.");
            entOptions.AddAllowedClass(typeof(Polyline3d), true);

            var entResult = ed.GetEntity(entOptions);
            if (entResult.Status != PromptStatus.OK) return;

            using (var tr = db.TransactionManager.StartTransaction())
            {
                var p3d = tr.GetObject(entResult.ObjectId, OpenMode.ForRead) as Polyline3d;
                if (p3d == null) return;

                // Bước 2: Dùng hàm tạo GeoPolycurve từ Polyline3d
                var geoPoly = new GeoPolycurve(p3d);
                var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);

                // Bước 3: Test tại các tham số t
                var tValues = new[] { 0.25, 0.5, 0.75 };

                foreach (var t in tValues)
                {
                    var coordSys = geoPoly.CoordinateSystemAtParameter(t);
                    var originPoint = coordSys.Origin;

                    // Vẽ một đường tròn tại gốc của hệ tọa độ
                    var circle = new Circle(originPoint, Vector3d.ZAxis, 1.0)
                    {
                        ColorIndex = 2 // Màu vàng
                    };

                    btr.AppendEntity(circle);
                    tr.AddNewlyCreatedDBObject(circle, true);
                }

                tr.Commit();
                ed.WriteMessage($"\n✅ Đã tạo thành công {tValues.Length} vòng tròn tại các điểm test.");
            }
        }

        [CommandMethod("BIMUP_GeoPolyCurve_PointAtParameter")]
        public void TestParameterAtPointOn3dPoly()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            if (doc == null) return;
            var db = doc.Database;
            var ed = doc.Editor;

            // Chọn một đối tượng 3D Polyline
            var entOptions = new PromptEntityOptions("\n👉 Chọn một đối tượng 3D Polyline:");
            entOptions.SetRejectMessage("\nĐối tượng phải là 3D Polyline.");
            entOptions.AddAllowedClass(typeof(Polyline3d), true);

            var entResult = ed.GetEntity(entOptions);
            if (entResult.Status != PromptStatus.OK) return;

            // Chọn một điểm để kiểm tra
            var ptResult = ed.GetPoint("\n👉 Chọn một điểm để tìm điểm gần nhất trên tuyến:");
            if (ptResult.Status != PromptStatus.OK) return;
            var testPoint = ptResult.Value;

            using (var tr = db.TransactionManager.StartTransaction())
            {
                var p3d = tr.GetObject(entResult.ObjectId, OpenMode.ForRead) as Polyline3d;
                if (p3d == null) return;

                var geoPoly = new GeoPolycurve(p3d);
                var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);

                // Sử dụng logic tương tự hàm ParameterAtPoint để tìm điểm gần nhất
                double bestT = geoPoly.ParameterAtPoint(testPoint);
                Point3d closestPointOnPolycurve = geoPoly.PointAtParameter(bestT);

                // Vẽ đường thẳng từ điểm chọn đến điểm gần nhất trên polyline
                var connectionLine = new Line(testPoint, closestPointOnPolycurve)
                {
                    ColorIndex = 1 // Màu Đỏ
                };
                btr.AppendEntity(connectionLine);
                tr.AddNewlyCreatedDBObject(connectionLine, true);

                // Vẽ thêm 1 vòng tròn tại điểm gần nhất để làm nổi bật
                var resultCircle = new Circle(closestPointOnPolycurve, Vector3d.ZAxis, 1.0)
                {
                    ColorIndex = 3 // Màu Lục
                };
                btr.AppendEntity(resultCircle);
                tr.AddNewlyCreatedDBObject(resultCircle, true);

                tr.Commit();
            }
        }
        #endregion

        #region GeoFace
        [CommandMethod("Test_GeoTriangle_LineTriangleIntersection")]
        public void Test_GeoTriangle_LineTriangleIntersection()
        {
            Document doc = Application.DocumentManager.MdiActiveDocument;
            Editor ed = doc.Editor;
            Database db = doc.Database;

            // --- Bước 1: Chọn đường Line ---
            ObjectIdCollection lineIds = SelectUtilities.SelectObjects("LINE", "\nChọn một đường Line: ");
            if (lineIds.Count == 0)
            {
                ed.WriteMessage("\nChưa chọn đối tượng nào.");
                return;
            }
            ObjectId lineId = lineIds[0]; // Lấy đối tượng đầu tiên được chọn

            // --- Bước 2: Chọn 3D Polyline (đại diện cho tam giác) ---
            ObjectIdCollection polyIds = SelectUtilities.SelectObjects("POLYLINE", "\nChọn 3D Polyline (3 đỉnh) làm tam giác: ");
            if (polyIds.Count == 0)
            {
                ed.WriteMessage("\nChưa chọn đối tượng nào.");
                return;
            }
            ObjectId polyId = polyIds[0]; // Lấy đối tượng đầu tiên được chọn

            using (Transaction tr = db.TransactionManager.StartTransaction())
            {
                try
                {
                    // Lấy các đối tượng AutoCAD
                    var acadLine = tr.GetObject(lineId, OpenMode.ForRead) as Line;
                    var acadPoly = tr.GetObject(polyId, OpenMode.ForRead) as Polyline3d;

                    if (acadLine == null)
                    {
                        ed.WriteMessage("\nĐối tượng chọn đầu tiên không phải là Line.");
                        return;
                    }
                    if (acadPoly == null)
                    {
                        ed.WriteMessage("\nĐối tượng chọn thứ hai không phải là 3D Polyline.");
                        return;
                    }

                    // Tạo các đối tượng Geo từ API
                    var geoLine = new GeoLine(acadLine.StartPoint, acadLine.EndPoint);
                    var triangle = GeoTriangle.FromPolyline3d(acadPoly);

                    if (triangle == null)
                    {
                        ed.WriteMessage("\n3D Polyline được chọn không hợp lệ (cần có đúng 3 đỉnh).");
                        return;
                    }

                    // --- Bước 3: Thực hiện kiểm tra giao ---
                    GeoCurves intersection = triangle.IntersectWith(geoLine);

                    // --- Bước 4: Vẽ kết quả ---
                    if (intersection != null)
                    {
                        Entity[] testEntities = intersection.Test();
                        if (testEntities != null && testEntities.Length > 0)
                        {
                            var blockTable = tr.GetObject(db.BlockTableId, OpenMode.ForRead) as BlockTable;
                            var modelSpace = tr.GetObject(blockTable[BlockTableRecord.ModelSpace], OpenMode.ForWrite) as BlockTableRecord;

                            foreach (var ent in testEntities)
                            {
                                ent.ColorIndex = 1; // Màu đỏ
                                modelSpace.AppendEntity(ent);
                                tr.AddNewlyCreatedDBObject(ent, true);
                            }
                            ed.WriteMessage("\nĐã tìm thấy và vẽ đối tượng giao.");
                        }
                    }
                    else
                    {
                        ed.WriteMessage("\nKhông tìm thấy giao điểm nào.");
                    }

                    tr.Commit();
                }
                catch (System.Exception ex)
                {
                    ed.WriteMessage($"\nLỗi: {ex.Message}");
                    tr.Abort();
                }
            }
        }

        [CommandMethod("Test_GeoTriangle_ArcTriangleIntersection")]
        public void Test_GeoTriangle_ArcTriangleIntersection()
        {
            Document doc = Application.DocumentManager.MdiActiveDocument;
            Editor ed = doc.Editor;
            Database db = doc.Database;

            // --- Bước 1: Chọn đường Arc ---
            ObjectIdCollection arcIds = SelectUtilities.SelectObjects("ARC", "\nChọn một đường Arc: ");
            if (arcIds.Count == 0)
            {
                ed.WriteMessage("\nChưa chọn đối tượng nào.");
                return;
            }
            ObjectId arcId = arcIds[0]; // Lấy đối tượng đầu tiên

            // --- Bước 2: Chọn 3D Polyline (đại diện cho tam giác) ---
            ObjectIdCollection polyIds = SelectUtilities.SelectObjects("POLYLINE", "\nChọn 3D Polyline (3 đỉnh) làm tam giác: ");
            if (polyIds.Count == 0)
            {
                ed.WriteMessage("\nChưa chọn đối tượng nào.");
                return;
            }
            ObjectId polyId = polyIds[0]; // Lấy đối tượng đầu tiên

            using (Transaction tr = db.TransactionManager.StartTransaction())
            {
                try
                {
                    // Lấy các đối tượng AutoCAD
                    var acadArc = tr.GetObject(arcId, OpenMode.ForRead) as Arc;
                    var acadPoly = tr.GetObject(polyId, OpenMode.ForRead) as Polyline3d;

                    if (acadArc == null)
                    {
                        ed.WriteMessage("\nĐối tượng chọn đầu tiên không phải là Arc.");
                        return;
                    }
                    if (acadPoly == null)
                    {
                        ed.WriteMessage("\nĐối tượng chọn thứ hai không phải là 3D Polyline.");
                        return;
                    }

                    // Tạo các đối tượng Geo từ API
                    var geoArc = new GeoArc(acadArc);
                    var triangle = GeoTriangle.FromPolyline3d(acadPoly);

                    if (triangle == null)
                    {
                        ed.WriteMessage("\n3D Polyline được chọn không hợp lệ (cần có đúng 3 đỉnh).");
                        return;
                    }

                    // --- Bước 3: Thực hiện kiểm tra giao ---
                    List<GeoCurves> intersections = triangle.IntersectWith(geoArc);

                    // --- Bước 4: Vẽ kết quả ---
                    if (intersections != null && intersections.Count > 0)
                    {
                        var blockTable = tr.GetObject(db.BlockTableId, OpenMode.ForRead) as BlockTable;
                        var modelSpace = tr.GetObject(blockTable[BlockTableRecord.ModelSpace], OpenMode.ForWrite) as BlockTableRecord;

                        foreach (var intersectionCurve in intersections)
                        {
                            Entity[] testEntities = intersectionCurve.Test();
                            if (testEntities != null)
                            {
                                foreach (var ent in testEntities)
                                {
                                    ent.ColorIndex = 1; // Màu đỏ
                                    modelSpace.AppendEntity(ent);
                                    tr.AddNewlyCreatedDBObject(ent, true);
                                }
                            }
                        }
                        ed.WriteMessage($"\nĐã tìm thấy và vẽ {intersections.Count} đối tượng giao.");
                    }
                    else
                    {
                        ed.WriteMessage("\nKhông tìm thấy giao điểm nào.");
                    }

                    tr.Commit();
                }
                catch (System.Exception ex)
                {
                    ed.WriteMessage($"\nLỗi: {ex.Message}");
                    tr.Abort();
                }
            }
        }
        #endregion
    }
}
