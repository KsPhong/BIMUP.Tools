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

        [CommandMethod("Test_GeoLine_Info")]
        public void Test_GeoLine_Info()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var ed = doc.Editor;
            var db = doc.Database;

            var res = ed.GetEntity("\n👉 Chọn một đối tượng LINE:");
            if (res.Status != PromptStatus.OK) return;

            using (var tr = db.TransactionManager.StartTransaction())
            {
                var line = tr.GetObject(res.ObjectId, OpenMode.ForRead) as Line;
                if (line == null)
                {
                    ed.WriteMessage("\n❌ Đối tượng không phải LINE.");
                    return;
                }

                var geoLine = new GeoLine(line.StartPoint, line.EndPoint);
                ed.WriteMessage(geoLine.GetInfoString());

                tr.Commit();
            }
        }

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

                    var geoLine = new GeoLine(acadLine);
                    var geoArc = new GeoArc(acadArc);

                    // Kiểm tra giao điểm
                    IntersectionResult intersection = geoLine.IntersectWith(geoArc);

                    if (intersection.Type == IntersectionResult.IntersectType.Intersect)
                    {

                        var blockTable = tr.GetObject(db.BlockTableId, OpenMode.ForRead) as BlockTable;
                        var modelSpace = tr.GetObject(blockTable[BlockTableRecord.ModelSpace], OpenMode.ForWrite) as BlockTableRecord;

                        int i = 1;
                        foreach (var pt in intersection.Points)
                        {
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
            var ed = doc.Editor;
            var db = doc.Database;

            var res = ed.GetEntity("\n👉 Chọn một đối tượng ARC:");
            if (res.Status != PromptStatus.OK) return;

            using (var tr = db.TransactionManager.StartTransaction())
            {
                var arc = tr.GetObject(res.ObjectId, OpenMode.ForRead) as Arc;
                if (arc == null)
                {
                    ed.WriteMessage("\n❌ Đối tượng không phải ARC.");
                    return;
                }

                try
                {
                    var geoArc = new GeoArc(arc); // ✅ dùng constructor gốc
                    ed.WriteMessage(geoArc.GetInfoString());
                }
                catch (System.Exception ex)
                {
                    ed.WriteMessage($"\n❌ Lỗi tạo GeoArc: {ex.Message}");
                }

                tr.Commit();
            }
        }

        [CommandMethod("Test_GeoArc_From3Point")]
        public void Test_GeoArc_From3Point()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var db = doc.Database;
            var ed = doc.Editor;

            try
            {
                // 1. Pick 3 điểm
                PromptPointResult res1 = ed.GetPoint("\nChọn điểm bắt đầu cung: ");
                if (res1.Status != PromptStatus.OK) return;

                PromptPointResult res2 = ed.GetPoint("\nChọn điểm giữa cung: ");
                if (res2.Status != PromptStatus.OK) return;

                PromptPointResult res3 = ed.GetPoint("\nChọn điểm kết thúc cung: ");
                if (res3.Status != PromptStatus.OK) return;

                Point3d p1 = res1.Value;
                Point3d p2 = res2.Value;
                Point3d p3 = res3.Value;

                // 2. Dựng GeoArc từ 3 điểm
                GeoArc arc = new GeoArc(p1, p2, p3);

                // 3. Gọi Test() để lấy Arc entity
                Entity[] entities = arc.Test();
                if (entities.Length == 0) return;

                // 4. Ghi ra model space
                using (Transaction tr = db.TransactionManager.StartTransaction())
                {
                    BlockTableRecord btr = (BlockTableRecord)tr.GetObject(
                        db.CurrentSpaceId, OpenMode.ForWrite);

                    foreach (Entity ent in entities)
                    {
                        btr.AppendEntity(ent);
                        tr.AddNewlyCreatedDBObject(ent, true);
                    }

                    tr.Commit();
                }
            }
            catch (System.Exception ex)
            {
                ed.WriteMessage($"\nLỗi: {ex.Message}");
            }
        }

        [CommandMethod("Test_GeoArc_Arc")]
        public void Test_GeoArc_Arc()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var db = doc.Database;
            var ed = doc.Editor;

            try
            {
                // 1. Cho người dùng chọn một đối tượng Arc
                var res = ed.GetEntity("\nChọn cung ARC: ");
                if (res.Status != PromptStatus.OK) return;

                using (var tr = db.TransactionManager.StartTransaction())
                {
                    var arc = tr.GetObject(res.ObjectId, OpenMode.ForRead) as Arc;
                    if (arc == null)
                    {
                        ed.WriteMessage("\nĐối tượng không phải là Arc.");
                        return;
                    }

                    // 2. Lấy 3 điểm từ Arc
                    Point3d start = arc.StartPoint;
                    Point3d mid = arc.GetPointAtDist(arc.Length / 2.0);
                    Point3d end = arc.EndPoint;

                    // 3. Dựng GeoArc từ 3 điểm
                    GeoArc geoArc = new GeoArc(start, mid, end);

                    // 4. Test → tạo entity để vẽ
                    Entity[] ents = geoArc.Test();
                    if (ents.Length == 0) return;

                    // 5. Ghi vào Model Space
                    BlockTableRecord btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);
                    foreach (var ent in ents)
                    {
                        btr.AppendEntity(ent);
                        tr.AddNewlyCreatedDBObject(ent, true);
                    }

                    tr.Commit();
                }
            }
            catch (System.Exception ex)
            {
                ed.WriteMessage($"\nLỗi: {ex.Message}");
            }
        }

        [CommandMethod("Test_GeoArc_CoordinateSystemAtParam")]
        public void Test_GeoArc_CoordinateSystemAtParam()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var db = doc.Database;
            var ed = doc.Editor;

            try
            {
                // B1: Chọn một Arc
                var arcRes = ed.GetEntity("\nChọn cung ARC: ");
                if (arcRes.Status != PromptStatus.OK) return;

                // B2: Nhập tham số t ∈ [0, 1]
                var tRes = ed.GetDouble("\nNhập tham số t (0.0 ~ 1.0): ");
                if (tRes.Status != PromptStatus.OK) return;

                double t = Math.Max(0.0, Math.Min(1.0, tRes.Value)); // Clamp [0,1]

                using (var tr = db.TransactionManager.StartTransaction())
                {
                    var arc = tr.GetObject(arcRes.ObjectId, OpenMode.ForRead) as Arc;
                    if (arc == null)
                    {
                        ed.WriteMessage("\nKhông phải là Arc.");
                        return;
                    }

                    // B3: Dựng GeoArc từ 3 điểm
                    var geoArc = new GeoArc(arc.StartPoint, arc.GetPointAtDist(arc.Length / 2.0), arc.EndPoint);

                    // B4: Lấy hệ tọa độ tại tham số t
                    GeoCoordinateSystem frame = geoArc.CoordinateSystemAtParameter(t);

                    // B5: Dùng .Test() để tạo các Line trục
                    var lines = frame.Test();

                    var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);
                    foreach (var ln in lines)
                    {
                        btr.AppendEntity(ln);
                        tr.AddNewlyCreatedDBObject(ln, true);
                    }

                    tr.Commit();
                }
            }
            catch (System.Exception ex)
            {
                ed.WriteMessage($"\nLỗi: {ex.Message}");
            }
        }

        [CommandMethod("Test_GeoArc_ParameterAtPoint")]
        public void Test_GeoArc_ParameterAtPoint()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var db = doc.Database;
            var ed = doc.Editor;

            try
            {
                var arcRes = ed.GetEntity("\nChọn cung ARC: ");
                if (arcRes.Status != PromptStatus.OK) return;

                var ptRes = ed.GetPoint("\nChọn một điểm bất kỳ: ");
                if (ptRes.Status != PromptStatus.OK) return;

                Point3d picked = ptRes.Value;

                using (var tr = db.TransactionManager.StartTransaction())
                {
                    var arc = tr.GetObject(arcRes.ObjectId, OpenMode.ForRead) as Arc;
                    if (arc == null)
                    {
                        ed.WriteMessage("\nKhông phải là Arc.");
                        return;
                    }

                    var geoArc = new GeoArc(arc.StartPoint, arc.GetPointAtDist(arc.Length / 2.0), arc.EndPoint);
                    double t = geoArc.ParameterAtPoint(picked);
                    Point3d ptOnArc = geoArc.PointAtParameter(t);

                    var circle = new Circle(ptOnArc, geoArc.Normal, 0.3)
                    {
                        ColorIndex = 2
                    };

                    var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);
                    btr.AppendEntity(circle);
                    tr.AddNewlyCreatedDBObject(circle, true);

                    ed.WriteMessage($"\nTham số t = {t:0.000}");

                    tr.Commit();
                }
            }
            catch (System.Exception ex)
            {
                ed.WriteMessage($"\nLỗi: {ex.Message}");
            }
        }

        [CommandMethod("Test_GeoArc_SplitByParameter")]
        public void Test_GeoArc_SplitByParameter()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var db = doc.Database;
            var ed = doc.Editor;

            try
            {
                // 1. Chọn cung ARC
                var arcRes = ed.GetEntity("\nChọn cung ARC để chia: ");
                if (arcRes.Status != PromptStatus.OK) return;

                // 2. Nhập danh sách tham số t (0–1), cách nhau bởi dấu phẩy
                var input = ed.GetString("\nNhập danh sách tham số t (ví dụ: 0.25,0.5,0.75): ");
                if (input.Status != PromptStatus.OK) return;

                var tList = input.StringResult
                    .Split(new[] { ',' }, StringSplitOptions.RemoveEmptyEntries)
                    .Select(s =>
                    {
                        if (double.TryParse(s.Trim(), out double val)) return val;
                        return -1.0;
                    })
                    .Where(t => t >= 0.0 && t <= 1.0)
                    .ToList();

                if (tList.Count == 0)
                {
                    ed.WriteMessage("\nKhông có tham số hợp lệ.");
                    return;
                }

                using (var tr = db.TransactionManager.StartTransaction())
                {
                    var arc = tr.GetObject(arcRes.ObjectId, OpenMode.ForRead) as Arc;
                    if (arc == null)
                    {
                        ed.WriteMessage("\nKhông phải là Arc.");
                        return;
                    }

                    // 3. Tạo GeoArc từ 3 điểm
                    var geoArc = new GeoArc(arc.StartPoint, arc.GetPointAtDist(arc.Length / 2.0), arc.EndPoint);

                    // 4. Gọi SplitByParameter
                    var splitSegments = geoArc.SplitByParameter(tList);

                    // 5. Vẽ từng đoạn ra model space
                    var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);
                    int color = 1;

                    foreach (GeoArc segment in splitSegments.OfType<GeoArc>())
                    {
                        foreach (var ent in segment.Test())
                        {
                            ent.ColorIndex = color;
                            btr.AppendEntity(ent);
                            tr.AddNewlyCreatedDBObject(ent, true);
                        }

                        color = (color % 7) + 1; // Đổi màu cho dễ phân biệt
                    }

                    tr.Commit();
                }
            }
            catch (System.Exception ex)
            {
                ed.WriteMessage($"\nLỗi: {ex.Message}");
            }
        }

        [CommandMethod("Test_GeoLine_IntersectWith_GeoArc")]
        public void Test_GeoLine_IntersectWith_GeoArc()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var db = doc.Database;
            var ed = doc.Editor;

            try
            {
                // 1. Chọn đối tượng ARC
                var arcRes = ed.GetEntity("\nChọn cung ARC: ");
                if (arcRes.Status != PromptStatus.OK) return;

                // 2. Chọn LINE hoặc Polyline có đúng 2 đỉnh
                var lineRes = ed.GetEntity("\nChọn đoạn thẳng (LINE hoặc Polyline 2 đỉnh): ");
                if (lineRes.Status != PromptStatus.OK) return;

                using (var tr = db.TransactionManager.StartTransaction())
                {
                    // 3. Dựng GeoArc
                    var arcObj = tr.GetObject(arcRes.ObjectId, OpenMode.ForRead) as Arc;
                    if (arcObj == null)
                    {
                        ed.WriteMessage("\n❌ Đối tượng không phải là Arc.");
                        return;
                    }

                    var geoArc = new GeoArc(
                        arcObj.StartPoint,
                        arcObj.GetPointAtDist(arcObj.Length / 2.0),
                        arcObj.EndPoint
                    );

                    // 4. Dựng GeoLine
                    var lineObj = tr.GetObject(lineRes.ObjectId, OpenMode.ForRead);
                    Point3d p1, p2;

                    if (lineObj is Line line)
                    {
                        p1 = line.StartPoint;
                        p2 = line.EndPoint;
                    }
                    else if (lineObj is Polyline pl && pl.NumberOfVertices == 2)
                    {
                        p1 = pl.GetPoint3dAt(0);
                        p2 = pl.GetPoint3dAt(1);
                    }
                    else
                    {
                        ed.WriteMessage("\n❌ Chỉ chấp nhận LINE hoặc Polyline có 2 đỉnh.");
                        return;
                    }

                    var geoLine = new GeoLine(p1, p2);

                    // 5. Gọi giao điểm theo đúng thiết kế: line.IntersectWith(arc)
                    var result = geoLine.IntersectWith(geoArc);

                    if (!result.HasIntersection)
                    {
                        ed.WriteMessage("\n❌ Không có giao điểm.");
                        return;
                    }

                    // 6. Vẽ các điểm giao
                    var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);
                    foreach (var pt in result.Points)
                    {
                        var circle = new Circle(pt, geoArc.Normal, 0.2)
                        {
                            ColorIndex = 1
                        };
                        btr.AppendEntity(circle);
                        tr.AddNewlyCreatedDBObject(circle, true);
                    }

                    ed.WriteMessage($"\n✅ Có {result.Points.Count} giao điểm.");
                    tr.Commit();
                }
            }
            catch (System.Exception ex)
            {
                ed.WriteMessage($"\nLỗi: {ex.Message}");
            }
        }

        [CommandMethod("Test_GeoArc_IntersectWith_GeoArc")]
        public void Test_GeoArc_IntersectWith_GeoArc()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var db = doc.Database;
            var ed = doc.Editor;

            try
            {
                // 1. Chọn ARC thứ nhất
                var arcRes1 = ed.GetEntity("\n👉 Chọn cung ARC thứ nhất: ");
                if (arcRes1.Status != PromptStatus.OK) return;

                // 2. Chọn ARC thứ hai
                var arcRes2 = ed.GetEntity("\n👉 Chọn cung ARC thứ hai: ");
                if (arcRes2.Status != PromptStatus.OK) return;

                using (var tr = db.TransactionManager.StartTransaction())
                {
                    // 3. Lấy đối tượng ARC thứ nhất
                    var arcObj1 = tr.GetObject(arcRes1.ObjectId, OpenMode.ForRead) as Arc;
                    if (arcObj1 == null)
                    {
                        ed.WriteMessage("\n❌ Đối tượng đầu tiên không phải là Arc.");
                        return;
                    }

                    var geoArc1 = new GeoArc(
                        arcObj1.StartPoint,
                        arcObj1.GetPointAtDist(arcObj1.Length / 2.0),
                        arcObj1.EndPoint
                    );

                    // 4. Lấy đối tượng ARC thứ hai
                    var arcObj2 = tr.GetObject(arcRes2.ObjectId, OpenMode.ForRead) as Arc;
                    if (arcObj2 == null)
                    {
                        ed.WriteMessage("\n❌ Đối tượng thứ hai không phải là Arc.");
                        return;
                    }

                    var geoArc2 = new GeoArc(
                        arcObj2.StartPoint,
                        arcObj2.GetPointAtDist(arcObj2.Length / 2.0),
                        arcObj2.EndPoint
                    );

                    // 5. Gọi hàm giao điểm
                    var result = geoArc1.IntersectWithArc(geoArc2);

                    if (!result.HasIntersection)
                    {
                        ed.WriteMessage("\n❌ Không có giao điểm giữa hai cung.");
                        return;
                    }

                    // 6. Vẽ điểm giao
                    var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);
                    foreach (var pt in result.Points)
                    {
                        var circle = new Circle(pt, geoArc1.Normal, 0.2)
                        {
                            ColorIndex = 2 // đỏ
                        };
                        btr.AppendEntity(circle);
                        tr.AddNewlyCreatedDBObject(circle, true);
                    }

                    ed.WriteMessage($"\n✅ Có {result.Points.Count} giao điểm.");
                    tr.Commit();
                }
            }
            catch (System.Exception ex)
            {
                ed.WriteMessage($"\nLỗi: {ex.Message}");
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

        [CommandMethod("Test_GeoPolycurve3d_IntersectWith")]
        public void Test_GeoPolycurve3d_IntersectWith()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var db = doc.Database;
            var ed = doc.Editor;

            try
            {
                // 1. Chọn Polyline3d thứ nhất
                var res1 = ed.GetEntity("\n👉 Chọn Polyline3D thứ nhất: ");
                if (res1.Status != PromptStatus.OK) return;

                // 2. Chọn Polyline3d thứ hai
                var res2 = ed.GetEntity("\n👉 Chọn Polyline3D thứ hai: ");
                if (res2.Status != PromptStatus.OK) return;

                using (var tr = db.TransactionManager.StartTransaction())
                {
                    var pl1 = tr.GetObject(res1.ObjectId, OpenMode.ForRead) as Polyline3d;
                    var pl2 = tr.GetObject(res2.ObjectId, OpenMode.ForRead) as Polyline3d;

                    if (pl1 == null || pl2 == null)
                    {
                        ed.WriteMessage("\n❌ Cả hai đối tượng phải là Polyline3D.");
                        return;
                    }

                    // 3. Dựng GeoPolycurve từ Polyline3d
                    GeoPolycurve geo1 = new GeoPolycurve(pl1);
                    GeoPolycurve geo2 = new GeoPolycurve(pl2);

                    // 4. Giao nhau
                    var result = geo1.IntersectWith(geo2);

                    if (!result.HasIntersection)
                    {
                        ed.WriteMessage("\n❌ Không có giao điểm.");
                        return;
                    }

                    // 5. Vẽ giao điểm
                    var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);
                    foreach (var pt in result.Points)
                    {
                        var circle = new Circle(pt, Vector3d.ZAxis, 0.2)
                        {
                            ColorIndex = 5 // màu xanh dương
                        };
                        btr.AppendEntity(circle);
                        tr.AddNewlyCreatedDBObject(circle, true);
                    }

                    ed.WriteMessage($"\n✅ Có {result.Points.Count} giao điểm.");
                    tr.Commit();
                }
            }
            catch (System.Exception ex)
            {
                ed.WriteMessage($"\nLỗi: {ex.Message}");
            }
        }

        [CommandMethod("Test_GeoPolycurve_Info")]
        public void Test_GeoPolycurve_Info()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var db = doc.Database;
            var ed = doc.Editor;

            var res = ed.GetEntity("\n👉 Chọn một Polyline3D để xem thông tin: ");
            if (res.Status != PromptStatus.OK) return;

            using (var tr = db.TransactionManager.StartTransaction())
            {
                var obj = tr.GetObject(res.ObjectId, OpenMode.ForRead) as Polyline3d;
                if (obj == null)
                {
                    ed.WriteMessage("\n❌ Không phải là Polyline3D.");
                    return;
                }

                var poly = new GeoPolycurve(obj);
                ed.WriteMessage(poly.PrintInfo());

                tr.Commit();
            }
        }

        [CommandMethod("Test_OverkillGeoLine")]
        public static void Test_OverkillGeoLine()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var ed = doc.Editor;
            var db = doc.Database;

            var res = ed.GetSelection();
            if (res.Status != PromptStatus.OK)
            {
                ed.WriteMessage("\n❌ Không có đối tượng nào được chọn.");
                return;
            }

            using (var tr = db.TransactionManager.StartTransaction())
            {
                var input = new List<GeoCurves>();
                var toErase = new List<ObjectId>();

                foreach (ObjectId id in res.Value.GetObjectIds())
                {
                    var ent = tr.GetObject(id, OpenMode.ForRead) as Entity;
                    if (ent == null) continue;

                    if (ent is Line line)
                    {
                        input.Add(new GeoLine(line));
                        toErase.Add(id); // ✔️ ghi nhớ để xóa
                    }
                    else if (ent is Polyline3d poly3d)
                    {
                        try
                        {
                            var geo = new GeoPolycurve(poly3d);
                            input.AddRange(geo.Segments.OfType<GeoLine>());
                            toErase.Add(id); // ✔️ ghi nhớ để xóa
                        }
                        catch (System.Exception ex)
                        {
                            ed.WriteMessage($"\n⚠️ Không thể chuyển Polyline3D: {ex.Message}");
                        }
                    }
                }

                if (input.Count == 0)
                {
                    ed.WriteMessage("\n⚠️ Không có đối tượng hợp lệ để xử lý.");
                    return;
                }

                // B2: Gọi OverKill
                var result = GeometryUtils.OverKill(input);

                // B3: Xóa các line/3dpolyline cũ
                foreach (var id in toErase)
                {
                    var obj = tr.GetObject(id, OpenMode.ForWrite);
                    obj?.Erase();
                }

                // B4: Vẽ kết quả
                var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);
                int color = 1;

                foreach (var poly in result)
                {
                    foreach (var ent in poly.Test())
                    {
                        ent.ColorIndex = color;
                        btr.AppendEntity(ent);
                        tr.AddNewlyCreatedDBObject(ent, true);
                    }
                    color = (color % 7) + 1;
                }

                tr.Commit();
                //ed.WriteMessage($"\n✅ Đã gộp và thay thế {toErase.Count} đường thành {result.Count} polyline.");
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

        [CommandMethod("Test_GeoTriangle_IsCoplanar")]
        public void Test_GeoTriangle_IsCoplanar()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var db = doc.Database;
            var ed = doc.Editor;

            try
            {
                // 1. Chọn tam giác thứ nhất (Polyline3d)
                var res1 = ed.GetEntity("\n👉 Chọn tam giác thứ nhất (Polyline3d): ");
                if (res1.Status != PromptStatus.OK) return;

                // 2. Chọn tam giác thứ hai (Polyline3d)
                var res2 = ed.GetEntity("\n👉 Chọn tam giác thứ hai (Polyline3d): ");
                if (res2.Status != PromptStatus.OK) return;

                using (var tr = db.TransactionManager.StartTransaction())
                {
                    var poly1 = tr.GetObject(res1.ObjectId, OpenMode.ForRead) as Polyline3d;
                    var poly2 = tr.GetObject(res2.ObjectId, OpenMode.ForRead) as Polyline3d;

                    var tri1 = GeoTriangle.FromPolyline3d(poly1);
                    var tri2 = GeoTriangle.FromPolyline3d(poly2);

                    if (tri1 == null || tri2 == null)
                    {
                        ed.WriteMessage("\n❌ Không thể tạo tam giác từ Polyline3d.");
                        return;
                    }

                    bool coplanar = tri1.IsCoplanarWith(tri2);
                    if (coplanar)
                        ed.WriteMessage("\n✅ Hai tam giác ĐỒNG PHẲNG.");
                    else
                        ed.WriteMessage("\n⚠️ Hai tam giác KHÔNG đồng phẳng.");

                    tr.Commit();
                }
            }
            catch (System.Exception ex)
            {
                ed.WriteMessage($"\nLỗi: {ex.Message}");
            }
        }

        [CommandMethod("Test_GeoTriangle_IntersectionLine")]
        public void Test_GeoTriangle_IntersectionLine()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var db = doc.Database;
            var ed = doc.Editor;

            try
            {
                // Chọn tam giác thứ nhất
                var res1 = ed.GetEntity("\n👉 Chọn Polyline3D làm tam giác thứ nhất: ");
                if (res1.Status != PromptStatus.OK) return;

                // Chọn tam giác thứ hai
                var res2 = ed.GetEntity("\n👉 Chọn Polyline3D làm tam giác thứ hai: ");
                if (res2.Status != PromptStatus.OK) return;

                using (var tr = db.TransactionManager.StartTransaction())
                {
                    var obj1 = tr.GetObject(res1.ObjectId, OpenMode.ForRead) as Polyline3d;
                    var obj2 = tr.GetObject(res2.ObjectId, OpenMode.ForRead) as Polyline3d;

                    if (obj1 == null || obj2 == null)
                    {
                        ed.WriteMessage("\n❌ Phải chọn Polyline3d.");
                        return;
                    }

                    var tri1 = GeoTriangle.FromPolyline3d(obj1);
                    var tri2 = GeoTriangle.FromPolyline3d(obj2);

                    if (tri1 == null || tri2 == null)
                    {
                        ed.WriteMessage("\n❌ Không tạo được GeoTriangle từ Polyline3d.");
                        return;
                    }

                    // Kiểm tra giao tuyến
                    GeoLine intersectionLine = tri1.GetIntersectionLineWith(tri2);
                    if (intersectionLine == null)
                    {
                        ed.WriteMessage("\n❌ Hai tam giác không giao nhau (không đồng phẳng hoặc song song).");
                        return;
                    }

                    // Vẽ đường giao tuyến
                    var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);
                    var acLine = new Line(intersectionLine.StartPoint, intersectionLine.EndPoint)
                    {
                        ColorIndex = 1 // đỏ
                    };

                    btr.AppendEntity(acLine);
                    tr.AddNewlyCreatedDBObject(acLine, true);

                    ed.WriteMessage("\n✅ Đã vẽ đường giao tuyến giữa hai tam giác.");
                    tr.Commit();
                }
            }
            catch (System.Exception ex)
            {
                ed.WriteMessage($"\n❌ Lỗi: {ex.Message}");
            }
        }

        [CommandMethod("Test_GeoTriangle_IntersectSegment")]
        public void Test_GeoTriangle_IntersectSegment()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var db = doc.Database;
            var ed = doc.Editor;

            try
            {
                // Chọn tam giác thứ nhất
                var res1 = ed.GetEntity("\n👉 Chọn tam giác đầu tiên (Polyline3D có 3 đỉnh): ");
                if (res1.Status != PromptStatus.OK) return;

                // Chọn tam giác thứ hai
                var res2 = ed.GetEntity("\n👉 Chọn tam giác thứ hai (Polyline3D có 3 đỉnh): ");
                if (res2.Status != PromptStatus.OK) return;

                using (var tr = db.TransactionManager.StartTransaction())
                {
                    var obj1 = tr.GetObject(res1.ObjectId, OpenMode.ForRead) as Polyline3d;
                    var obj2 = tr.GetObject(res2.ObjectId, OpenMode.ForRead) as Polyline3d;

                    var tri1 = GeoTriangle.FromPolyline3d(obj1);
                    var tri2 = GeoTriangle.FromPolyline3d(obj2);

                    if (tri1 == null || tri2 == null)
                    {
                        ed.WriteMessage("\n❌ Một trong hai polyline không hợp lệ hoặc không có đúng 3 đỉnh.");
                        return;
                    }

                    // Gọi hàm kiểm tra giao nhau
                    var intersectSegment = tri1.GetIntersection(tri2);

                    if (intersectSegment == null)
                    {
                        ed.WriteMessage("\n❌ Không có đoạn giao nhau giữa hai tam giác.");
                        return;
                    }

                    // Vẽ đoạn giao
                    var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);
                    var line = new Line(intersectSegment.StartPoint, intersectSegment.EndPoint)
                    {
                        ColorIndex = 1
                    };
                    btr.AppendEntity(line);
                    tr.AddNewlyCreatedDBObject(line, true);

                    ed.WriteMessage("\n✅ Hai tam giác giao nhau! Đoạn giao đã được vẽ.");
                    tr.Commit();
                }
            }
            catch (System.Exception ex)
            {
                ed.WriteMessage($"\nLỗi: {ex.Message}");
            }
        }

        [CommandMethod("Test_TriangulateGeoPolycurve")]
        public void Test_TriangulateGeoPolycurve()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var db = doc.Database;
            var ed = doc.Editor;

            var res = ed.GetEntity("\n👉 Chọn một Polyline3D kín để chia tam giác: ");
            if (res.Status != PromptStatus.OK) return;

            using (var tr = db.TransactionManager.StartTransaction())
            {
                var obj = tr.GetObject(res.ObjectId, OpenMode.ForRead) as Polyline3d;
                if (obj == null)
                {
                    ed.WriteMessage("\n❌ Không phải là Polyline3D.");
                    return;
                }

                var poly = new GeoPolycurve(obj);
                var triangles = poly.Triangulate(); // ✅ dùng phương thức instance

                if (triangles.Count == 0)
                {
                    ed.WriteMessage("\n⚠️ Không thể chia tam giác.");
                    return;
                }

                var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);

                foreach (var tri in triangles)
                {
                    var entities = tri.Test(); // sử dụng hàm Test() của GeoTriangle
                    foreach (var ent in entities)
                    {
                        btr.AppendEntity(ent);
                        tr.AddNewlyCreatedDBObject(ent, true);
                    }
                }

                ed.WriteMessage($"\n✅ Đã tạo {triangles.Count} tam giác.");
                tr.Commit();
            }
        }

        #endregion



    }
}
