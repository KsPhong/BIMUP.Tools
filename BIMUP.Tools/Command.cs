using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Autodesk.AutoCAD.ApplicationServices;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.EditorInput;
using Autodesk.AutoCAD.Runtime;
using System.IO;
using System.Windows.Forms;
using BIMUP.API;
using Application = Autodesk.AutoCAD.ApplicationServices.Application;
using Autodesk.AutoCAD.Geometry;

namespace BIMUP.Core
{
    public static class Command
    {
        /// <summary>
        /// Lệnh chính: gọi các bước con để cắt khối Solid3d bằng các Polyline khép kín.
        /// </summary>
        [CommandMethod("BIMUP-CUTSOLID")]
        public static void CutSolid()
        {
            var doc = Application.DocumentManager.MdiActiveDocument;
            var ed = doc.Editor;
            var db = doc.Database;

            if (!TrySelectEntities(ed, out var plineIds, out var solidIds)) return;

            using (Transaction tr = db.TransactionManager.StartTransaction())
            {
                var btr = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);

                double maxH = CalcMaxHeight(solidIds, tr);
                var cutters = BuildCutters(plineIds, maxH, ed, tr);

                if (cutters.Count == 0)
                {
                    ed.WriteMessage("\n❌ Không tạo được khối cắt.");
                    return;
                }

                SubtractSolids(solidIds, cutters, btr, tr, ed);

                tr.Commit();
                ed.WriteMessage("\n✅ Hoàn tất subtract !");
            }
        }

        /// <summary>
        /// Chọn các Polyline và Solid3D từ người dùng bằng SelectionFilter.
        /// </summary>
        private static bool TrySelectEntities(Editor ed,
                                              out ObjectIdCollection plineIds,
                                              out ObjectIdCollection solidIds)
        {
            plineIds = new ObjectIdCollection();
            solidIds = new ObjectIdCollection();

            // Chọn polyline
            var plineFilter = new SelectionFilter(new[] {
                new TypedValue((int)DxfCode.Start, "LWPOLYLINE")
            });

            var plineResult = ed.GetSelection(new PromptSelectionOptions
            {
                MessageForAdding = "\nChọn các Polyline khép kín để tạo khối cắt:"
            }, plineFilter);

            if (plineResult.Status != PromptStatus.OK)
            {
                ed.WriteMessage("\n❌ Không có Polyline nào được chọn.");
                return false;
            }

            foreach (var id in plineResult.Value.GetObjectIds())
                plineIds.Add(id);

            // Chọn solid
            var solidFilter = new SelectionFilter(new[] {
                new TypedValue((int)DxfCode.Start, "3DSOLID")
            });

            var solidResult = ed.GetSelection(new PromptSelectionOptions
            {
                MessageForAdding = "\nChọn các Solid3D bị cắt:"
            }, solidFilter);

            if (solidResult.Status != PromptStatus.OK)
            {
                ed.WriteMessage("\n❌ Không có Solid3D nào được chọn.");
                return false;
            }

            foreach (var id in solidResult.Value.GetObjectIds())
                solidIds.Add(id);

            return true;
        }

        /// <summary>
        /// Tính chiều cao lớn nhất của tập Solid3D.
        /// </summary>
        private static double CalcMaxHeight(ObjectIdCollection solids, Transaction tr)
        {
            return solids.Cast<ObjectId>()
                         .Select(id => tr.GetObject(id, OpenMode.ForRead) as Solid3d)
                         .Where(s => s != null)
                         .Max(s => s.GeometricExtents.MaxPoint.Z - s.GeometricExtents.MinPoint.Z)
                         + 100.0; // Dự phòng
        }

        /// <summary>
        /// Tạo các khối cắt từ danh sách polyline khép kín.
        /// </summary>
        private static List<Solid3d> BuildCutters(ObjectIdCollection plines,
                                                  double maxH,
                                                  Editor ed,
                                                  Transaction tr)
        {
            var list = new List<Solid3d>();

            foreach (ObjectId id in plines)
            {
                var pl = tr.GetObject(id, OpenMode.ForRead) as Polyline;
                if (pl == null || !pl.Closed)
                {
                    ed.WriteMessage("\n⚠ Polyline không khép kín => bỏ qua.");
                    continue;
                }

                var curves = new DBObjectCollection { (DBObject)pl.Clone() };
                var regions = Region.CreateFromCurves(curves);
                if (regions.Count == 0)
                {
                    ed.WriteMessage("\n⚠ Không tạo được Region.");
                    continue;
                }

                using (Region reg = regions[0] as Region)
                {
                    reg.TransformBy(Matrix3d.Displacement(-Vector3d.ZAxis * maxH));
                    var cut = new Solid3d();
                    cut.SetDatabaseDefaults();
                    cut.Extrude(reg, 2 * maxH, 0);
                    list.Add(cut);
                }
            }

            return list;
        }

        /// <summary>
        /// Thực hiện subtract cho từng solid mục tiêu bằng toàn bộ cutters.
        /// </summary>
        private static void SubtractSolids(ObjectIdCollection targets,
                                           List<Solid3d> cutters,
                                           BlockTableRecord btr,
                                           Transaction tr,
                                           Editor ed)
        {
            foreach (ObjectId tgtId in targets)
            {
                var tgt = tr.GetObject(tgtId, OpenMode.ForWrite) as Solid3d;
                if (tgt == null) continue;

                foreach (var cutter in cutters)
                {
                    try
                    {
                        var tmp = cutter.Clone() as Solid3d;
                        btr.AppendEntity(tmp);
                        tr.AddNewlyCreatedDBObject(tmp, true);

                        tgt.BooleanOperation(BooleanOperationType.BoolSubtract, tmp);
                        tmp.Erase();
                    }
                    catch (System.Exception ex)
                    {
                        ed.WriteMessage($"\n⚠ Lỗi khi cắt: {ex.Message}");
                    }
                }
            }
        }
    }
}
