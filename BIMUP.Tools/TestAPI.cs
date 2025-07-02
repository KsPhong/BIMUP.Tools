using Autodesk.AutoCAD.ApplicationServices;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.EditorInput;
using Autodesk.AutoCAD.Geometry;
using Autodesk.AutoCAD.Runtime;
using BIMUP.API;
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
        #endregion

        #region GeoArc

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
                catch (Exception ex)
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

        [CommandMethod("Test_GeoArc_By3Points")]
        public void Test_GeoArc_By3Points()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var db = doc.Database;
            var ed = doc.Editor;

            // Pick 3 điểm: start, mid, end
            var p1 = ed.GetPoint("\n🟡 Chọn điểm START:");
            if (p1.Status != PromptStatus.OK) return;

            var p2 = ed.GetPoint("\n🟡 Chọn điểm MID:");
            if (p2.Status != PromptStatus.OK) return;

            var p3 = ed.GetPoint("\n🟡 Chọn điểm END:");
            if (p3.Status != PromptStatus.OK) return;

            try
            {
                // Dựng GeoArc từ 3 điểm
                var geo = new GeoArc(p1.Value, p2.Value, p3.Value);

                // Tạo Arc từ dữ liệu GeoArc
                var arc = new Arc(
                    geo.CenterPoint,
                    geo.Normal,
                    geo.Radius,
                    geo.StartAngle,
                    geo.EndAngle
                )
                {
                    ColorIndex = 1 // Đỏ
                };

                using (var tr = db.TransactionManager.StartTransaction())
                {
                    var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);
                    btr.AppendEntity(arc);
                    tr.AddNewlyCreatedDBObject(arc, true);
                    tr.Commit();
                }

                ed.WriteMessage("\n✅ Tạo cung thành công!");
            }
            catch (Exception ex)
            {
                ed.WriteMessage($"\n❌ Lỗi: {ex.Message}");
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
            catch (Exception ex)
            {
                ed.WriteMessage($"\n❌ Lỗi: {ex.Message}");
            }
        }

        #endregion



    }
}
